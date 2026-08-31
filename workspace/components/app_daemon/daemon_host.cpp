#include "daemon_host.hpp"

#include <cinttypes>

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

namespace background {
namespace {

constexpr const char* TAG = "daemon";

// Warn once past this much of a daemon's declared stack. Sizing the stack is
// the app's own call -- this only makes sure the call is an informed one.
constexpr uint32_t kStackWarnPercent = 75;

// Idle wait when the daemon declares no tick period. Capped rather than
// infinite so the task stays inside the watchdog window and keeps saying it is
// alive.
constexpr uint32_t kIdleWaitMs = kMaxSleepMs;

uint32_t nowMsFromTimer()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

UBaseType_t clampPriority(const char* id, UBaseType_t wanted)
{
    if (wanted < kMinPriority) {
        ESP_LOGW(TAG, "'%s' xin uu tien %u -> nang len %u (san toi thieu)", id,
                 static_cast<unsigned>(wanted),
                 static_cast<unsigned>(kMinPriority));
        return kMinPriority;
    }
    if (wanted > kMaxPriority) {
        // The one that matters. Above kMaxPriority a daemon outranks the UI
        // task and its background work starts costing the foreground its
        // frames -- the exact arrangement this component exists to end.
        ESP_LOGW(TAG,
                 "'%s' xin uu tien %u -> HA xuong %u: daemon khong duoc cao hon "
                 "task UI (6)",
                 id, static_cast<unsigned>(wanted),
                 static_cast<unsigned>(kMaxPriority));
        return kMaxPriority;
    }
    return wanted;
}

}  // namespace

// --------------------------------------------------------------- registration

bool DaemonHost::add(IAppDaemon* d)
{
    if (d == nullptr || d->id() == nullptr) {
        return false;
    }
    if (started_) {
        ESP_LOGE(TAG, "'%s': them sau khi da start -- tu choi", d->id());
        return false;
    }
    if (count_ >= kMaxDaemons) {
        ESP_LOGE(TAG, "'%s': het cho (%u/%u)", d->id(),
                 static_cast<unsigned>(count_),
                 static_cast<unsigned>(kMaxDaemons));
        return false;
    }
    // A duplicate would get two tasks and two inboxes for one object, and every
    // number it keeps would be counted twice. Same reason SampleFanout::add()
    // refuses one.
    for (uint8_t i = 0; i < count_; ++i) {
        if (slots_[i].daemon == d) {
            ESP_LOGE(TAG, "'%s': da dang ky roi", d->id());
            return false;
        }
    }

    Slot& s  = slots_[count_];
    s.daemon = d;
    s.cfg    = d->config();
    s.host   = this;
    ++count_;
    return true;
}

// ---------------------------------------------------------------------- start

esp_err_t DaemonHost::start()
{
    if (started_) {
        return ESP_ERR_INVALID_STATE;
    }
    started_ = true;

    for (uint8_t i = 0; i < count_; ++i) {
        Slot&       s  = slots_[i];
        const char* id = s.daemon->id();

        // A daemon with neither samples nor a tick is called once, at
        // onStart(), and never again. That is almost always a forgotten field
        // rather than an intention, and the symptom -- a feature that quietly
        // does nothing -- is the hardest kind to trace back to its cause.
        if (!s.cfg.wantsSamples && s.cfg.tickMs == 0) {
            ESP_LOGW(TAG,
                     "'%s': wantsSamples=false va tickMs=0 -- sau onStart() se "
                     "khong bao gio duoc goi lai. Co phai y ban khong?",
                     id);
        }

        if (s.cfg.wantsSamples) {
            if (s.cfg.inboxDepth == 0) {
                s.cfg.inboxDepth = 1;
            }
            s.inbox = xQueueCreate(s.cfg.inboxDepth, sizeof(Envelope));
            if (s.inbox == nullptr) {
                ESP_LOGE(TAG, "'%s': khong tao duoc hop thu %u x %u byte", id,
                         static_cast<unsigned>(s.cfg.inboxDepth),
                         static_cast<unsigned>(sizeof(Envelope)));
                continue;  // the others still start; partial failure is normal
            }
        }

        s.cfg.priority = clampPriority(id, s.cfg.priority);

        // Named after the daemon on purpose. A stack overflow or a watchdog
        // report prints the TASK name, and that name is the only thing standing
        // between "the board rebooted" and "app X rebooted the board".
        if (xTaskCreate(&DaemonHost::trampoline, id, s.cfg.stackSize, &s,
                        s.cfg.priority, &s.task) != pdPASS) {
            ESP_LOGE(TAG, "'%s': khong tao duoc task (stack %" PRIu32 " byte)",
                     id, s.cfg.stackSize);
            if (s.inbox != nullptr) {
                vQueueDelete(s.inbox);
                s.inbox = nullptr;
            }
            s.task = nullptr;
            continue;
        }

        ESP_LOGI(TAG,
                 "'%s' chay: stack %" PRIu32 " byte, uu tien %u, hop thu %u, "
                 "tick %" PRIu32 " ms",
                 id, s.cfg.stackSize, static_cast<unsigned>(s.cfg.priority),
                 static_cast<unsigned>(s.cfg.wantsSamples ? s.cfg.inboxDepth : 0),
                 s.cfg.tickMs);
    }

    return ESP_OK;
}

uint8_t DaemonHost::running() const
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < count_; ++i) {
        if (slots_[i].task != nullptr) {
            ++n;
        }
    }
    return n;
}

// -------------------------------------------------------- the producer's side

void DaemonHost::onSample(const sensors::Sample& sample, uint32_t nowMs)
{
    // RUNS IN THE IMU TASK, priority 10. Everything here is bounded and
    // non-blocking: one struct copy per interested daemon, and a counter when a
    // queue is full. No app code, no allocation, no lock held across a call.
    Envelope e{sample, nowMs};

    for (uint8_t i = 0; i < count_; ++i) {
        Slot& s = slots_[i];
        if (s.inbox == nullptr) {
            continue;
        }
        // Zero wait, always. Blocking here would hand a slow daemon the power
        // to stall the sensor task, which is the failure this whole component
        // exists to make impossible.
        if (xQueueSend(s.inbox, &e, 0) != pdTRUE) {
            ++s.dropped;
        }
    }
}

// -------------------------------------------------------- the daemon's side

void DaemonHost::trampoline(void* arg)
{
    Slot* s = static_cast<Slot*>(arg);
    s->host->taskBody(*s);
}

void DaemonHost::taskBody(Slot& slot)
{
    // Subscribing to the task watchdog is what turns "a daemon hung and the
    // watch feels broken" into a log line naming the daemon.
    // CONFIG_ESP_TASK_WDT_PANIC is off, so the report is printed and the board
    // keeps running: the other daemons, the UI and the drivers are untouched.
    const bool watched = (esp_task_wdt_add(nullptr) == ESP_OK);

    slot.daemon->onStart();

    const uint32_t waitMs =
        (slot.cfg.tickMs > 0)
            ? (slot.cfg.tickMs < kMaxSleepMs ? slot.cfg.tickMs : kMaxSleepMs)
            : kIdleWaitMs;
    uint32_t nextTickMs = nowMsFromTimer() + slot.cfg.tickMs;

    for (;;) {
        if (watched) {
            esp_task_wdt_reset();
        }

        if (slot.inbox != nullptr) {
            Envelope e{};
            if (xQueueReceive(slot.inbox, &e, pdMS_TO_TICKS(waitMs)) == pdTRUE) {
                // Age first: how long this sample sat in the queue waiting for
                // this task to be given a core. Measured before the callback so
                // the callback's own cost cannot inflate it.
                const uint32_t now = nowMsFromTimer();
                const uint32_t age = (now > e.nowMs) ? (now - e.nowMs) : 0;
                if (age > slot.maxAgeMs) {
                    slot.maxAgeMs = age;
                }
                slot.ageSumMs += age;
                ++slot.ageCount;

                const int64_t t0 = esp_timer_get_time();
                slot.daemon->onSample(e.sample, e.nowMs);
                const uint32_t us =
                    static_cast<uint32_t>(esp_timer_get_time() - t0);
                if (us > slot.maxWallUs) {
                    slot.maxWallUs = us;
                }
                ++slot.received;
            }
        } else {
            // Timer-only daemon: nothing to wait on but the clock.
            vTaskDelay(pdMS_TO_TICKS(waitMs));
        }

        if (slot.cfg.tickMs > 0) {
            const uint32_t now = nowMsFromTimer();
            // Signed difference, so a wrapped millisecond clock does not turn
            // into a tick every iteration for the next 49 days.
            if (static_cast<int32_t>(now - nextTickMs) >= 0) {
                nextTickMs       = now + slot.cfg.tickMs;
                const int64_t t0 = esp_timer_get_time();
                slot.daemon->onTick(now);
                const uint32_t us =
                    static_cast<uint32_t>(esp_timer_get_time() - t0);
                if (us > slot.maxWallUs) {
                    slot.maxWallUs = us;
                }
                ++slot.ticks;
            }
        }
    }
}

// ---------------------------------------------------------------- diagnostics

void DaemonHost::logDiagnostics(const char* why)
{
    if (count_ == 0) {
        return;
    }
    ESP_LOGI(TAG, "--- daemon (%s) --- %u/%u dang chay", why,
             static_cast<unsigned>(running()), static_cast<unsigned>(count_));

    for (uint8_t i = 0; i < count_; ++i) {
        Slot& s = slots_[i];
        if (s.task == nullptr) {
            ESP_LOGW(TAG, "  %-14s KHONG CHAY", s.daemon->id());
            continue;
        }

        // ESP-IDF returns this in bytes, not words.
        const uint32_t freeStack =
            static_cast<uint32_t>(uxTaskGetStackHighWaterMark(s.task));
        const uint32_t used =
            (s.cfg.stackSize > freeStack) ? (s.cfg.stackSize - freeStack) : 0;
        const uint32_t usedPct =
            (s.cfg.stackSize > 0) ? (used * 100 / s.cfg.stackSize) : 0;

        const uint32_t avgAge =
            (s.ageCount > 0) ? (s.ageSumMs / s.ageCount) : 0;

        ESP_LOGI(TAG,
                 "  %-14s nhan=%" PRIu32 " rot=%" PRIu32 " tick=%" PRIu32
                 "  stack con=%" PRIu32 "/%" PRIu32 " byte (dung %" PRIu32 "%%)",
                 s.daemon->id(), s.received, s.dropped, s.ticks, freeStack,
                 s.cfg.stackSize, usedPct);

        // The two numbers that must be read together. Age is how long the
        // daemon was made to wait; wall is how long one callback took INCLUDING
        // any preemption inside it. A big wall with a small age is slow code; a
        // big age is a task that is not being given a core.
        ESP_LOGI(TAG,
                 "    tuoi mau: tb %" PRIu32 " ms, max %" PRIu32
                 " ms   |   dong ho treo trong callback: max %" PRIu32 " us",
                 avgAge, s.maxAgeMs, s.maxWallUs);

        // Once. A warning that repeats every ten seconds is a warning people
        // learn to filter out, and this one has to survive being read months
        // later.
        if (!s.stackWarned && usedPct >= kStackWarnPercent) {
            s.stackWarned = true;
            ESP_LOGW(TAG,
                     "  %s da dung %" PRIu32 "%% stack cua chinh no. Tang "
                     "DaemonConfig::stackSize -- tran stack lam reset board va "
                     "bao cao se mang ten task nay.",
                     s.daemon->id(), usedPct);
        }
        if (s.dropped > 0) {
            // Deliberately does not say "onSample() is slow" any more. The
            // first version did, and it was wrong: the callback was cheap and
            // the task was starved. A diagnostic that names the wrong cause is
            // worse than one that names none.
            ESP_LOGW(TAG,
                     "  %s rot %" PRIu32 "/%" PRIu32 " mau. Xem tuoi mau o tren: "
                     "tuoi cao = task bi bo doi (daemon nam duoi UI, dung y do); "
                     "tuoi thap ma van rot = onSample() cham that.",
                     s.daemon->id(), s.dropped, s.dropped + s.received);
        }
    }
}

}  // namespace background
