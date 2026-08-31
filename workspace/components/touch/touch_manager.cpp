#include "touch_manager.hpp"

#include <cinttypes>
#include <new>

#include "esp_log.h"
#include "esp_timer.h"

namespace touch {

namespace {

constexpr const char* TAG = "touch";

uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

}  // namespace

const char* toString(State s)
{
    switch (s) {
    case State::Stopped:      return "Stopped";
    case State::Starting:     return "Starting";
    case State::Idle:         return "Idle";
    case State::Tracking:     return "Tracking";
    case State::ChipSleeping: return "ChipSleeping";
    case State::Recovering:   return "Recovering";
    case State::Fault:        return "Fault";
    case State::Stopping:     return "Stopping";
    }
    return "?";
}

// ------------------------------------------------------------------ locking

bool TouchManager::lock() const
{
    if (mutex_ == nullptr) {
        return false;
    }
    // Never portMAX_DELAY. A wait without a ceiling is not expressible anywhere
    // else in this project and must not appear here either.
    return xSemaphoreTake(mutex_, pdMS_TO_TICKS(kLockTimeoutMs)) == pdTRUE;
}

void TouchManager::unlock() const
{
    if (mutex_ != nullptr) {
        xSemaphoreGive(mutex_);
    }
}

void TouchManager::setState(State s)
{
    const State old = state_.exchange(s, std::memory_order_relaxed);
    if (old != s) {
        ESP_LOGD(TAG, "state %s -> %s", toString(old), toString(s));
    }
}

// --------------------------------------------------------------------- init

esp_err_t TouchManager::init(i2c::Device& device, const Wiring& wiring,
                             const Behavior& behavior, const Geometry& geometry)
{
    if (started_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!device.valid()) {
        ESP_LOGE(TAG, "i2c::Device chua duoc tao");
        return ESP_ERR_INVALID_ARG;
    }
    if (wiring.resetPin == wiring.intPin) {
        ESP_LOGE(TAG, "RST va INT khong the la cung mot chan");
        return ESP_ERR_INVALID_ARG;
    }

    device_    = &device;
    wiring_    = wiring;
    behavior_  = behavior;
    geometry_  = geometry;

    // A synthetic release can only fire on a tick, so ticking slower than the
    // timeout turns the timeout into the tick interval. Caught here rather
    // than left to show up as "the button sometimes stays pressed".
    if (behavior_.idleTickMs >= behavior_.tracker.releaseTimeoutMs) {
        ESP_LOGW(TAG, "idleTickMs=%" PRIu32 " >= releaseTimeoutMs=%" PRIu32
                      " -- nha tay se tre hon mong doi",
                 behavior_.idleTickMs, behavior_.tracker.releaseTimeoutMs);
    }

    // Reset line first, parked INACTIVE. Configuring it as an output with the
    // wrong initial level holds the chip in reset from boot, and the symptom is
    // a panel that never answers anything.
    const int deasserted = wiring_.resetActiveLow ? 1 : 0;
    gpio_config_t rst{};
    rst.pin_bit_mask = 1ULL << wiring_.resetPin;
    rst.mode         = GPIO_MODE_OUTPUT;
    rst.pull_up_en   = GPIO_PULLUP_DISABLE;
    rst.pull_down_en = GPIO_PULLDOWN_DISABLE;
    rst.intr_type    = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&rst);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(RST GPIO%d): %s", static_cast<int>(wiring_.resetPin),
                 esp_err_to_name(err));
        return err;
    }
    gpio_set_level(wiring_.resetPin, deasserted);

    gpio_config_t irq{};
    irq.pin_bit_mask = 1ULL << wiring_.intPin;
    irq.mode         = GPIO_MODE_INPUT;
    irq.pull_up_en   = (wiring_.intInternalPullup && wiring_.intActiveLow)
                           ? GPIO_PULLUP_ENABLE
                           : GPIO_PULLUP_DISABLE;
    irq.pull_down_en = (wiring_.intInternalPullup && !wiring_.intActiveLow)
                           ? GPIO_PULLDOWN_ENABLE
                           : GPIO_PULLDOWN_DISABLE;
    // Edge, not level. Every interrupt source in the register document is a
    // low PULSE of 1..5 ms; a level trigger would re-enter for the width of
    // the pulse.
    irq.intr_type = wiring_.intActiveLow ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;

    err = gpio_config(&irq);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(INT GPIO%d): %s", static_cast<int>(wiring_.intPin),
                 esp_err_to_name(err));
        return err;
    }
    gpio_intr_disable(wiring_.intPin);

    if (mutex_ == nullptr) {
        // A real mutex, so it carries priority inheritance. The IDF I2C bus
        // lock is a binary semaphore and does not -- see i2c-bus-design.md 8.1.
        // Here the UI task (6) and the touch task (11) share a structure, and
        // without inheritance the UI could hold the touch task off for as long
        // as anything in between wanted the CPU.
        mutex_ = xSemaphoreCreateMutex();
        if (mutex_ == nullptr) {
            ESP_LOGE(TAG, "khong tao duoc mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    if (chip_ == nullptr) {
        chip_ = new (devStorage_) Cst816t(*device_);
    }

    TrackerConfig tcfg = behavior_.tracker;
    tracker_.configure(tcfg);

    setState(State::Stopped);
    return ESP_OK;
}

// -------------------------------------------------------------------- reset

void TouchManager::pulseReset()
{
    const int asserted   = wiring_.resetActiveLow ? 0 : 1;
    const int deasserted = wiring_.resetActiveLow ? 1 : 0;

    gpio_set_level(wiring_.resetPin, asserted);
    vTaskDelay(pdMS_TO_TICKS(behavior_.reset.assertMs));
    gpio_set_level(wiring_.resetPin, deasserted);
    vTaskDelay(pdMS_TO_TICKS(behavior_.reset.settleMs));

    ++diag_.resets;
}

// Reset, identify, configure -- IN THAT ORDER and all inside the window that
// follows a reset, because that is the only time the chip is guaranteed to be
// in dynamic mode and answering. Two seconds later it is in standby and every
// one of these writes would NACK.
esp_err_t TouchManager::resetAndConfigure(bool logResult)
{
    const uint32_t t0 = nowMs();

    pulseReset();

    // Identification NEVER gates anything. No document states an expected
    // ChipID, and Espressif records parts that fail this read while touch works
    // perfectly. It is logged because knowing which silicon is on the bench is
    // worth a lot when a log arrives from the field.
    diag_.chip = chip_->identify();
    if (logResult) {
        if (!diag_.chip.read) {
            ESP_LOGW(TAG, "khong doc duoc ChipID (0x%02X..0x%02X) -- KHONG phai loi, "
                          "van tiep tuc",
                     reg::IDENT_BASE, reg::IDENT_BASE + reg::IDENT_LEN - 1);
        } else {
            ESP_LOGI(TAG, "ChipID=0x%02X ProjID=0x%02X FW=0x%02X Factory=0x%02X%s",
                     diag_.chip.chipId, diag_.chip.projId, diag_.chip.fwVersion,
                     diag_.chip.factoryId,
                     diag_.chip.implausible() ? "  (toan 0x00/0xFF -- dang ngo)" : "");
            if (!diag_.chip.implausible()
                && diag_.chip.chipId != reg::CHIP_ID_SEEN_ON_V2_1) {
                // Not an error. Recorded because a differing ID is the first
                // thing worth knowing when behaviour differs between boards.
                ESP_LOGW(TAG, "  ChipID khac voi board V2.1 da do (0x%02X) -- khong "
                              "phai loi, chi de doi chieu khi so sanh log",
                         reg::CHIP_ID_SEEN_ON_V2_1);
            }
        }
    }

    diag_.lastConfig = chip_->applyConfig(behavior_.chip);

    if (logResult) {
        for (std::size_t i = 0; i < diag_.lastConfig.count; ++i) {
            const ConfigResult::Entry& e = diag_.lastConfig.entries[i];
            if (!e.writeOk) {
                ESP_LOGW(TAG, "  0x%02X <- 0x%02X  GHI THAT BAI", e.regAddr, e.wrote);
            } else if (behavior_.chip.verifyWrites && !e.verified) {
                ESP_LOGW(TAG, "  0x%02X <- 0x%02X  doc lai duoc 0x%02X", e.regAddr,
                         e.wrote, e.readBack);
            } else {
                ESP_LOGI(TAG, "  0x%02X <- 0x%02X  OK", e.regAddr, e.wrote);
            }
        }
    }

    const uint32_t elapsed = nowMs() - t0;
    if (elapsed > behavior_.reset.configWindowMs) {
        // The chip may already have slid into standby half way through, which
        // makes any failure above meaningless rather than diagnostic.
        ESP_LOGW(TAG, "cau hinh mat %" PRIu32 " ms, qua cua so %" PRIu32
                      " ms -- ket qua tren khong dang tin",
                 elapsed, behavior_.reset.configWindowMs);
    }

    if (!diag_.lastConfig.allWritesOk()) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (behavior_.chip.verifyWrites && !diag_.lastConfig.allVerified()) {
        ESP_LOGW(TAG, "thanh ghi 0x%02X khong doc lai dung gia tri da ghi",
                 diag_.lastConfig.firstMismatch());
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

// -------------------------------------------------------------------- start

esp_err_t TouchManager::start()
{
    if (started_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (chip_ == nullptr || device_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    setState(State::Starting);

    // Configured HERE, on the caller's task, before the touch task exists.
    // i2c_device.hpp calls this out as the legitimate exception to
    // one-task-one-device: a driver is set up from app_main and only then
    // handed over. The task claims ownership as its first action.
    //
    // Not fatal on failure. A chip that refuses configuration may still report
    // coordinates, and a watch that boots without touch is better than one
    // that does not boot.
    const esp_err_t cfgErr = resetAndConfigure(true);
    if (cfgErr != ESP_OK) {
        ESP_LOGW(TAG, "cau hinh ban dau that bai (%s) -- van chay, se thu phuc hoi",
                 esp_err_to_name(cfgErr));
    }

    stopRequested_.store(false, std::memory_order_relaxed);

    if (stopSemaphore_ == nullptr) {
        stopSemaphore_ = xSemaphoreCreateBinary();
    }
    if (commandDone_ == nullptr) {
        commandDone_ = xSemaphoreCreateBinary();
    }
    if (selfTestDone_ == nullptr) {
        selfTestDone_ = xSemaphoreCreateBinary();
    }
    if (stopSemaphore_ == nullptr || commandDone_ == nullptr || selfTestDone_ == nullptr) {
        ESP_LOGE(TAG, "khong tao duoc semaphore");
        releaseResources();
        return ESP_ERR_NO_MEM;
    }

    // The task must exist before the ISR is registered: the handler notifies
    // taskHandle_, and a handler installed first could fire into a null one.
    if (xTaskCreatePinnedToCore(taskFunc, "touch", behavior_.taskStackSize, this,
                                behavior_.taskPriority, &taskHandle_,
                                behavior_.taskCoreId) != pdPASS) {
        ESP_LOGE(TAG, "khong tao duoc task");
        releaseResources();
        return ESP_ERR_NO_MEM;
    }

    if (wiring_.installIsrService) {
        const esp_err_t err = gpio_install_isr_service(0);
        if (err == ESP_ERR_INVALID_STATE) {
            isrServiceOwned_ = false;  // someone got there first: fine
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(err));
            haltTask();
            releaseResources();
            return err;
        } else {
            isrServiceOwned_ = true;
        }
    }

    esp_err_t err = gpio_isr_handler_add(wiring_.intPin, isrHandler, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add(GPIO%d): %s", static_cast<int>(wiring_.intPin),
                 esp_err_to_name(err));
        haltTask();
        releaseResources();
        return err;
    }
    isrRegistered_ = true;
    gpio_intr_enable(wiring_.intPin);

    started_ = true;
    setState(State::Idle);

    ESP_LOGI(TAG, "san sang: 0x%02X, RST GPIO%d, INT GPIO%d, uu tien %d, IRQ_CTL=0x%02X",
             device_->address(), static_cast<int>(wiring_.resetPin),
             static_cast<int>(wiring_.intPin), static_cast<int>(behavior_.taskPriority),
             Cst816t::irqCtlByte(behavior_.chip));

    if (!behavior_.chip.disableAutoSleep) {
        ESP_LOGI(TAG, "auto-standby BAT (mac dinh): chip se im lang tren bus sau ~2 s "
                      "khong cham -- day la BINH THUONG, khong phai loi");
    } else {
        ESP_LOGW(TAG, "auto-standby TAT: chip luon tra loi I2C nhung ton ~1.6 mA thay vi "
                      "~6 uA. Chi dung khi dang do dac tren ban.");
    }
    return ESP_OK;
}

// --------------------------------------------------------------------- stop

void TouchManager::stop()
{
    if (!started_) {
        return;
    }
    setState(State::Stopping);

    // ISR FIRST, and this order is the whole point. A late edge arriving after
    // the task is deleted would notify a freed handle; removing the handler
    // first makes that impossible.
    gpio_intr_disable(wiring_.intPin);
    if (isrRegistered_) {
        gpio_isr_handler_remove(wiring_.intPin);
        isrRegistered_ = false;
    }

    haltTask();

    // A UI must never be left holding a widget down because the driver went
    // away. The transition stays in the queue so the UI can still drain it --
    // which is why mutex_ outlives stop() and is only freed in the destructor.
    if (lock()) {
        tracker_.cancel(nowMs());
        unlock();
    }

    releaseResources();
    started_ = false;
    setState(State::Stopped);
}

// Ask the task to leave its loop and WAIT until it has. releaseResources()
// deletes the task outright, and a task deleted mid-loop can be holding the
// mutex -- which would then never be given back -- or sitting inside an I2C
// transfer that owns the shared bus lock.
void TouchManager::haltTask()
{
    if (taskHandle_ == nullptr) {
        return;
    }
    stopRequested_.store(true, std::memory_order_relaxed);
    xTaskNotifyGive(taskHandle_);
    if (stopSemaphore_ != nullptr
        && xSemaphoreTake(stopSemaphore_, pdMS_TO_TICKS(kStopTimeoutMs)) != pdTRUE) {
        ESP_LOGW(TAG, "task khong dung trong %" PRIu32 " ms", kStopTimeoutMs);
    }
}

void TouchManager::releaseResources()
{
    if (taskHandle_ != nullptr) {
        vTaskDelete(taskHandle_);
        taskHandle_ = nullptr;
    }
    if (stopSemaphore_ != nullptr) {
        vSemaphoreDelete(stopSemaphore_);
        stopSemaphore_ = nullptr;
    }
    if (commandDone_ != nullptr) {
        vSemaphoreDelete(commandDone_);
        commandDone_ = nullptr;
    }
    if (selfTestDone_ != nullptr) {
        vSemaphoreDelete(selfTestDone_);
        selfTestDone_ = nullptr;
    }
    if (isrServiceOwned_) {
        gpio_uninstall_isr_service();
        isrServiceOwned_ = false;
    }
}

TouchManager::~TouchManager()
{
    stop();
    if (chip_ != nullptr) {
        chip_->~Cst816t();
        chip_ = nullptr;
    }
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

// ---------------------------------------------------------------------- ISR

void IRAM_ATTR TouchManager::isrHandler(void* arg)
{
    auto* self = static_cast<TouchManager*>(arg);

    self->irqCount_.fetch_add(1, std::memory_order_relaxed);
    // esp_timer_get_time() is IRAM-safe. Stamping here rather than in the task
    // is what makes interrupt-to-read latency a measurement instead of a guess
    // -- and that number is the only justification for priority 11.
    self->lastIrqUs_.store(esp_timer_get_time(), std::memory_order_relaxed);

    // Nothing else. No I2C, no logging, no application callback: the bus needs
    // a task context and the rest would run with interrupts disabled.
    BaseType_t hpTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(self->taskHandle_, &hpTaskWoken);
    if (hpTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

bool TouchManager::intActive() const
{
    const int level = gpio_get_level(wiring_.intPin);
    return wiring_.intActiveLow ? (level == 0) : (level != 0);
}

void TouchManager::maskIrq()
{
    gpio_intr_disable(wiring_.intPin);
}

void TouchManager::unmaskIrq()
{
    gpio_intr_enable(wiring_.intPin);
}

// --------------------------------------------------------------------- task

void TouchManager::taskFunc(void* arg)
{
    static_cast<TouchManager*>(arg)->run();
}

void TouchManager::run()
{
    // From here on this task is the only one allowed to touch the device.
    // i2c::Device logs an error if anyone else tries.
    device_->claimOwnership();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(behavior_.idleTickMs));

        if (stopRequested_.load(std::memory_order_relaxed)) {
            break;
        }

        const uint32_t now = nowMs();

        if (requestedMode_.load(std::memory_order_relaxed) != appliedMode_) {
            applyMode(now);
        }

        if (selfTestRequested_.exchange(false, std::memory_order_relaxed)) {
            performSelfTest(selfTestWaitMs_);
            xSemaphoreGive(selfTestDone_);
        }

        if (recoveryRequested_.exchange(false, std::memory_order_relaxed)) {
            enterRecovery("yeu cau tu ung dung", now);
        }

        const State st = state_.load(std::memory_order_relaxed);

        if (st == State::Fault) {
            // Backoff, not a spin. Without this, an interrupt line stuck active
            // resets the chip forever: reset, immediate interrupt, failed read,
            // reset again.
            if (static_cast<int32_t>(now - faultUntilMs_) >= 0) {
                recoveryAttempts_ = 0;
                enterRecovery("het thoi gian backoff", now);
            }
            continue;
        }

        if (st == State::ChipSleeping) {
            continue;  // the chip is parked; nothing to read and nothing to tick
        }

        serviceInterrupts(now);
        checkInterruptStuck();

        // Runs on every wake-up, interrupt or tick. This is what releases a
        // finger that lifted without the chip ever saying so.
        if (lock()) {
            tracker_.tick(now);
            const bool pressed = tracker_.snapshot().pressed;
            unlock();
            const State cur = state_.load(std::memory_order_relaxed);
            if (cur == State::Idle && pressed) {
                setState(State::Tracking);
            } else if (cur == State::Tracking && !pressed) {
                setState(State::Idle);
            }
        }
    }

    setState(State::Stopping);
    if (stopSemaphore_ != nullptr) {
        xSemaphoreGive(stopSemaphore_);
    }
    vTaskSuspend(nullptr);  // stop() deletes this task
}

// ------------------------------------------------------- interrupt servicing

void TouchManager::serviceInterrupts(uint32_t now)
{
    const uint32_t seen = irqCount_.load(std::memory_order_relaxed);
    if (seen == servicedIrq_) {
        // A plain tick. Reading here would poke a chip that is very likely in
        // standby, and the NACK would look like a fault.
        return;
    }

    // Several edges can land before this task runs. They fold into one wake-up
    // by design -- a queue per edge would simply overflow during a drag.
    diag_.irqCoalesced += (seen - servicedIrq_ - 1);
    servicedIrq_ = seen;

    const int64_t  irqUs   = lastIrqUs_.load(std::memory_order_relaxed);
    const uint32_t startMs = now;

    for (uint8_t i = 0; i < behavior_.maxReadsPerWake; ++i) {
        const int64_t readStartUs = esp_timer_get_time();
        const uint32_t latencyUs =
            (readStartUs > irqUs) ? static_cast<uint32_t>(readStartUs - irqUs) : 0;

        uint8_t     raw[reg::FRAME_LEN]{};
        bool        busOk = false;
        ParsedFrame frame = chip_->readFrame(geometry_.limits, busOk, raw);

        ++diag_.reads;

        if (!busOk) {
            // Only reads that FOLLOW an interrupt count towards ill health. The
            // chip declining to answer while idle is standby, not a fault.
            ++diag_.readFailures;
            ++consecutiveReadFailures_;
            if (consecutiveReadFailures_ >= behavior_.failuresBeforeRecovery) {
                enterRecovery("doc that bai lien tiep sau IRQ", now);
            }
            return;
        }

        consecutiveReadFailures_ = 0;
        handleFrame(frame, now, raw, latencyUs);

        // Stop unless more interrupts arrived while this read was in flight.
        const uint32_t again = irqCount_.load(std::memory_order_relaxed);
        if (again == servicedIrq_) {
            break;
        }
        servicedIrq_ = again;

        // Time ceiling as well as a count. i2c::kXferTimeoutMs is 50 ms and
        // applies to every transfer, so a wedged bus turns "four more reads"
        // into 200 ms of the shared bus held at priority 11.
        if ((nowMs() - startMs) >= behavior_.readBudgetMs) {
            break;
        }
    }

    // Deliberately NO interrupt-level check here. It was tried, and it was
    // wrong: 286 hits out of 418 interrupts on a panel with nothing whatsoever
    // wrong with it.
    //
    // The reason is 0xED. This chip drives a fixed-width LOW PULSE of 1..5 ms
    // and does NOT release the line because the frame was read -- it is not a
    // level that an acknowledgement clears. Measured on V2.1: the interrupt
    // reaches this task in ~32 us and a six-byte read at 400 kHz takes ~250 us,
    // so the line is still legitimately low roughly 300 us into a 1000 us
    // pulse, essentially every time. Checking here can only report a fault
    // that is not there.
    //
    // A stuck line is detected in run() instead, where the question is the
    // right one: is INT still asserted long after the last edge, with no new
    // edge coming? See checkInterruptStuck().

    if (irqToReadSamples_ > 0) {
        diag_.irqToReadAvgUs = static_cast<uint32_t>(irqToReadTotalUs_ / irqToReadSamples_);
    }
    if (contactIrqSamples_ > 0) {
        diag_.contactIrqAvgMs = static_cast<uint32_t>(contactIrqTotalMs_ / contactIrqSamples_);
    }
}

// The honest stuck-interrupt test: is the line STILL asserted long after the
// last edge, with no new edge arriving?
//
// The pulse the chip emits is 1..5 ms wide (0xED), so anything inside that
// window proves nothing. Past it, an asserted line with a frozen interrupt
// counter means nobody is going to produce the edge this driver is waiting
// for -- and since the ISR is edge triggered, that state never resolves on its
// own. That is the wake-loop the design has to defend against.
void TouchManager::checkInterruptStuck()
{
    // Pulse width plus a millisecond of margin. irqPulseByte() clamps to the
    // range the T document actually gives, so this follows a reconfiguration.
    const uint32_t guardUs =
        (static_cast<uint32_t>(Cst816t::irqPulseByte(behavior_.chip)) + 1) * 1000u;

    const int64_t sinceIrqUs = esp_timer_get_time() - lastIrqUs_.load(std::memory_order_relaxed);

    if (!intActive() || sinceIrqUs < static_cast<int64_t>(guardUs)) {
        intStuckTicks_ = 0;
        return;
    }

    ++diag_.intStuckActive;
    if (intStuckTicks_ < 0xFF) {
        ++intStuckTicks_;
    }

    // Several consecutive ticks, not one. idleTickMs is 40 ms by default, so
    // this is roughly a fifth of a second of a line that is asserted and going
    // nowhere.
    if (intStuckTicks_ >= kIntStuckTicksBeforeRecovery) {
        intStuckTicks_ = 0;
        enterRecovery("INT ket o muc tich cuc", nowMs());
    }
}

void TouchManager::countFrame(const ParsedFrame& frame)
{
    switch (frame.status) {
    case FrameStatus::Valid:          ++diag_.validFrames; break;
    case FrameStatus::NoFinger:       break;  // not an error and not a touch
    case FrameStatus::AllOnes:        ++diag_.allOnesFrames;  ++diag_.invalidFrames; break;
    case FrameStatus::BadFingerCount: ++diag_.badFingerCount; ++diag_.invalidFrames; break;
    case FrameStatus::ReservedEvent:  ++diag_.reservedEvent;  ++diag_.invalidFrames; break;
    case FrameStatus::OutOfRange:     ++diag_.outOfRange;     ++diag_.invalidFrames; break;
    }
    if (frame.edgeClamped)         ++diag_.edgeClamped;
    if (frame.gestureUnknown)      ++diag_.unknownGesture;
    if (frame.fingerEventMismatch) ++diag_.fingerEventMismatch;
}

void TouchManager::handleFrame(const ParsedFrame& frame, uint32_t now, const uint8_t* raw,
                               uint32_t latencyUs)
{
    countFrame(frame);

    irqToReadTotalUs_ += latencyUs;
    ++irqToReadSamples_;
    if (latencyUs > diag_.irqToReadMaxUs) {
        diag_.irqToReadMaxUs = latencyUs;
    }

    // Interval between interrupts DURING A CONTACT. This is the measurement
    // that turns TrackerConfig::releaseTimeoutMs from a guess into a number:
    // the timeout wants to sit comfortably above contactIrqMaxMs.
    if (frame.status == FrameStatus::Valid) {
        if (lastContactIrqMs_ != 0) {
            const uint32_t gap = now - lastContactIrqMs_;
            if (gap < 2000) {  // a longer gap is a new gesture, not a sample
                contactIrqTotalMs_ += gap;
                ++contactIrqSamples_;
                if (gap > diag_.contactIrqMaxMs) diag_.contactIrqMaxMs = gap;
                if (diag_.contactIrqMinMs == 0 || gap < diag_.contactIrqMinMs) {
                    diag_.contactIrqMinMs = gap;
                }
            }
        }
        lastContactIrqMs_ = now;
    } else if (frame.status == FrameStatus::NoFinger) {
        lastContactIrqMs_ = 0;
    }

    TouchPoint point{};
    if (frame.status == FrameStatus::Valid) {
        point = toLogical(frame.rawX, frame.rawY, geometry_.orientation);
        if (!point.valid) {
            // parseFrame accepted it against the raw panel size but the
            // orientation rejected it. That means the two disagree about the
            // panel, which is a configuration bug worth counting rather than
            // silently dropping.
            ++diag_.transformRejected;
        }
    }

    if (behavior_.logFrames) {
        const uint32_t windowMs = 1000;
        if (now - logWindowStartMs_ >= windowMs) {
            logWindowStartMs_ = now;
            logWindowCount_   = 0;
        }
        if (logWindowCount_ < behavior_.logFramesPerSecond) {
            ++logWindowCount_;
            // Two shapes, because printing "-> (0,0) REJECTED" for a NoFinger
            // frame accuses the transform of something it was never asked to
            // do. A release carries stale coordinates by design; only a Valid
            // frame is ever handed to toLogical().
            if (frame.status == FrameStatus::Valid) {
                ESP_LOGI(TAG,
                         "raw %02X %02X %02X %02X %02X %02X | %-14s f=%u ev=%-8s "
                         "raw(%u,%u) -> (%d,%d)%s | ges=%s | tre %" PRIu32 " us",
                         raw[0], raw[1], raw[2], raw[3], raw[4], raw[5],
                         toString(frame.status), frame.fingers, toString(frame.event),
                         frame.rawX, frame.rawY, static_cast<int>(point.x),
                         static_cast<int>(point.y),
                         point.valid ? "" : "  TRANSFORM TU CHOI",
                         gestureName(frame.gestureCode), latencyUs);
            } else {
                ESP_LOGI(TAG,
                         "raw %02X %02X %02X %02X %02X %02X | %-14s f=%u ev=%-8s "
                         "| ges=%s | tre %" PRIu32 " us",
                         raw[0], raw[1], raw[2], raw[3], raw[4], raw[5],
                         toString(frame.status), frame.fingers, toString(frame.event),
                         gestureName(frame.gestureCode), latencyUs);
            }
        }
    }

    if (lock()) {
        tracker_.onFrame(frame, point, now);
        unlock();
    }

    // Wake whoever drains this queue. From the touch TASK, not an ISR, so the
    // plain xTaskNotifyGive is correct here.
    if (wakeTarget_ != nullptr) {
        xTaskNotifyGive(wakeTarget_);
    }
}

// ----------------------------------------------------------------- recovery

void TouchManager::enterRecovery(const char* why, uint32_t now)
{
    if (recoveryAttempts_ >= behavior_.maxRecoveryAttempts) {
        ESP_LOGE(TAG, "phuc hoi that bai %u lan (%s) -> Fault, thu lai sau %" PRIu32 " ms",
                 recoveryAttempts_, why, behavior_.recoveryBackoffMs);
        ++diag_.recoveryFailures;
        faultUntilMs_ = now + behavior_.recoveryBackoffMs;

        // Release before going quiet, or the UI keeps a widget pressed for as
        // long as the fault lasts.
        if (lock()) {
            tracker_.cancel(now);
            unlock();
        }
        maskIrq();
        setState(State::Fault);
        return;
    }

    ++recoveryAttempts_;
    ++diag_.recoveries;
    setState(State::Recovering);
    ESP_LOGW(TAG, "phuc hoi lan %u: %s", recoveryAttempts_, why);

    // Interrupt off for the whole sequence: the reset itself can produce an
    // edge, and servicing it half way through reconfiguration reads a chip
    // whose registers are partly written.
    maskIrq();

    // Any contact in progress is over as far as the UI is concerned.
    if (lock()) {
        tracker_.cancel(now);
        unlock();
    }

    // ONLY the touch chip. The shared bus carries the IMU and the RTC, and
    // recovering it here would take two working devices down with this one.
    // If SDA/SCL are genuinely stuck, diagnostics say so and the application
    // calls BusManager::recover() -- see i2c-bus-design.md section 9.
    const esp_err_t err = resetAndConfigure(true);

    consecutiveReadFailures_ = 0;
    servicedIrq_             = irqCount_.load(std::memory_order_relaxed);
    lastContactIrqMs_        = 0;

    unmaskIrq();

    if (err == ESP_OK) {
        recoveryAttempts_ = 0;
        // Start from Released, never from the state before the fault: reusing
        // the old coordinate would place a press wherever the finger last was.
        setState(State::Idle);
        ESP_LOGI(TAG, "phuc hoi xong");
    } else {
        ESP_LOGW(TAG, "phuc hoi chua thanh cong: %s", esp_err_to_name(err));
        setState(State::Idle);  // the next failed read escalates again
    }
}

esp_err_t TouchManager::requestRecovery()
{
    if (!started_) {
        return ESP_ERR_INVALID_STATE;
    }
    recoveryRequested_.store(true, std::memory_order_relaxed);
    xTaskNotifyGive(taskHandle_);
    return ESP_OK;
}

// -------------------------------------------------------------------- modes

void TouchManager::applyMode(uint32_t now)
{
    const Mode want = requestedMode_.load(std::memory_order_relaxed);

    if (want == Mode::ChipSleep) {
        if (lock()) {
            tracker_.cancel(now);
            unlock();
        }
        maskIrq();
        if (chip_->enterDeepSleep()) {
            ESP_LOGI(TAG, "chip vao sleep (0xE5=0x03) -- chi reset moi danh thuc duoc");
            setState(State::ChipSleeping);
        } else {
            // Very likely standby, not a fault: the chip stopped answering
            // before it could be told to sleep. It is already in a low-power
            // state, so the outcome is nearly what was asked for.
            ESP_LOGW(TAG, "khong ghi duoc lenh sleep -- chip co the da o standby");
            setState(State::ChipSleeping);
        }
    } else {
        // Nothing but a reset brings the chip back from 0xE5 sleep.
        ESP_LOGI(TAG, "danh thuc chip bang reset");
        maskIrq();
        resetAndConfigure(true);
        servicedIrq_      = irqCount_.load(std::memory_order_relaxed);
        lastContactIrqMs_ = 0;
        unmaskIrq();
        setState(State::Idle);
    }

    appliedMode_ = want;
    if (commandDone_ != nullptr) {
        xSemaphoreGive(commandDone_);
    }
}

esp_err_t TouchManager::requestMode(Mode mode, uint32_t timeoutMs)
{
    if (!started_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (requestedMode_.load(std::memory_order_relaxed) == mode && appliedMode_ == mode) {
        return ESP_OK;
    }

    // The application never calls the driver: it asks the owning task and waits
    // for an acknowledgement. That is what keeps one-task-one-device true even
    // across a power transition.
    xSemaphoreTake(commandDone_, 0);  // drain a stale acknowledgement
    requestedMode_.store(mode, std::memory_order_relaxed);
    xTaskNotifyGive(taskHandle_);

    if (xSemaphoreTake(commandDone_, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) {
        ESP_LOGW(TAG, "doi xac nhan doi mode qua %" PRIu32 " ms", timeoutMs);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

// ----------------------------------------------------------------- UI-facing

void TouchManager::checkSingleConsumer()
{
    TaskHandle_t self = xTaskGetCurrentTaskHandle();

    if (consumer_ == nullptr) {
        consumer_ = self;  // first caller claims the queue
        return;
    }
    if (consumer_ == self || consumerWarned_) {
        return;
    }

    consumerWarned_ = true;
    ESP_LOGE(TAG,
             "hang doi cham bi RUT TU HAI TASK ('%s' va '%s'). popTransition() "
             "TIEU THU, nen ben nay an mat su kien cua ben kia -- mot cai Up bi "
             "cuop di se lam LVGL ket o trang thai dang cham cho toi lan cham ke "
             "tiep. Chi UI task duoc pop.",
             pcTaskGetName(consumer_), pcTaskGetName(self));
}

bool TouchManager::popTransition(TouchTransition& out)
{
    checkSingleConsumer();

    if (!lock()) {
        return false;
    }
    const bool got = tracker_.pop(out);
    unlock();
    return got;
}

TouchSnapshot TouchManager::snapshot() const
{
    TouchSnapshot s{};
    if (lock()) {
        s = tracker_.snapshot();
        unlock();
    }
    return s;
}

// -------------------------------------------------------------- diagnostics

void TouchManager::diagnostics(Diagnostics& out) const
{
    // The counters are written by the touch task without the lock, because
    // taking it on every frame would put the UI task in the path of the hot
    // loop. Each field is an aligned 32-bit word, so every value read here is a
    // real one -- but the SET may be a few microseconds out of step with
    // itself. That is the right trade for numbers whose purpose is to be
    // eyeballed in a log.
    out          = diag_;
    out.state    = state_.load(std::memory_order_relaxed);
    out.irqCount = irqCount_.load(std::memory_order_relaxed);

    if (lock()) {
        out.downs             = tracker_.downCount();
        out.ups               = tracker_.upCount();
        out.moves             = tracker_.moveCount();
        out.syntheticUps      = tracker_.syntheticUpCount();
        out.coalescedMoves    = tracker_.coalescedMoveCount();
        out.queueOverflows    = tracker_.overflowCount();
        out.missedUps         = tracker_.missedUpCount();
        out.eventUpWithFinger = tracker_.eventUpWithFingerCount();
        unlock();
    }
}

void TouchManager::logDiagnostics(const char* where) const
{
    Diagnostics d{};
    diagnostics(d);

    ESP_LOGI(TAG, "--- chan doan cam ung (%s) ---", where ? where : "");
    ESP_LOGI(TAG, "  state=%s  IRQ=%" PRIu32 " (gop %" PRIu32 ")  doc=%" PRIu32
                  "  loi doc=%" PRIu32,
             toString(d.state), d.irqCount, d.irqCoalesced, d.reads, d.readFailures);
    ESP_LOGI(TAG, "  frame: hop le=%" PRIu32 "  hong=%" PRIu32 "  (0xFF=%" PRIu32
                  " finger=%" PRIu32 " reserved=%" PRIu32 " ngoai bien=%" PRIu32 ")",
             d.validFrames, d.invalidFrames, d.allOnesFrames, d.badFingerCount,
             d.reservedEvent, d.outOfRange);
    ESP_LOGI(TAG, "  keo bien=%" PRIu32 "  transform tu choi=%" PRIu32
                  "  gesture la=%" PRIu32 "  finger/event lech=%" PRIu32,
             d.edgeClamped, d.transformRejected, d.unknownGesture, d.fingerEventMismatch);
    ESP_LOGI(TAG, "  su kien: Down=%" PRIu32 " Up=%" PRIu32 " (synthetic=%" PRIu32
                  ") Move=%" PRIu32 " (gop=%" PRIu32 ")",
             d.downs, d.ups, d.syntheticUps, d.moves, d.coalescedMoves);
    ESP_LOGI(TAG, "  tran hang doi=%" PRIu32 "  mat Up=%" PRIu32 "  event Up khi con "
                  "finger=%" PRIu32,
             d.queueOverflows, d.missedUps, d.eventUpWithFinger);
    ESP_LOGI(TAG, "  reset=%" PRIu32 "  phuc hoi=%" PRIu32 "  that bai=%" PRIu32
                  "  INT ket=%" PRIu32,
             d.resets, d.recoveries, d.recoveryFailures, d.intStuckActive);
    ESP_LOGI(TAG, "  tre IRQ->doc: tb %" PRIu32 " us, max %" PRIu32 " us",
             d.irqToReadAvgUs, d.irqToReadMaxUs);

    // The line that retires a guess. releaseTimeoutMs should sit above
    // contactIrqMaxMs with room to spare.
    ESP_LOGI(TAG, "  khoang IRQ khi dang cham: min %" PRIu32 " ms, tb %" PRIu32
                  " ms, MAX %" PRIu32 " ms  (releaseTimeoutMs dang la %" PRIu32 " ms)",
             d.contactIrqMinMs, d.contactIrqAvgMs, d.contactIrqMaxMs,
             behavior_.tracker.releaseTimeoutMs);

    if (d.chip.read) {
        ESP_LOGI(TAG, "  chip: ID=0x%02X Proj=0x%02X FW=0x%02X Factory=0x%02X",
                 d.chip.chipId, d.chip.projId, d.chip.fwVersion, d.chip.factoryId);
    } else {
        ESP_LOGI(TAG, "  chip: khong doc duoc identity (khong phai loi)");
    }
}

// ------------------------------------------------------------------ selftest

esp_err_t TouchManager::runSelfTest(SelfTestResult& out, uint32_t waitForTouchMs)
{
    if (!started_) {
        return ESP_ERR_INVALID_STATE;
    }

    selfTestWaitMs_ = waitForTouchMs;
    xSemaphoreTake(selfTestDone_, 0);
    selfTestRequested_.store(true, std::memory_order_relaxed);
    xTaskNotifyGive(taskHandle_);

    // Generous: the register work includes a reset, and the optional step waits
    // for a human.
    const uint32_t budget = waitForTouchMs + 2000;
    if (xSemaphoreTake(selfTestDone_, pdMS_TO_TICKS(budget)) != pdTRUE) {
        ESP_LOGE(TAG, "self test khong tra ket qua trong %" PRIu32 " ms", budget);
        return ESP_ERR_TIMEOUT;
    }

    out = selfTestResult_;
    return ESP_OK;
}

// Runs ON THE TOUCH TASK. Everything here talks to the chip, and this device
// has exactly one owner.
void TouchManager::performSelfTest(uint32_t waitForTouchMs)
{
    SelfTestResult r{};

    ESP_LOGI(TAG, "--- self test cam ung ---");

    maskIrq();
    const esp_err_t cfgErr = resetAndConfigure(true);
    unmaskIrq();

    r.resetDone         = true;
    r.identityRead      = diag_.chip.read;
    r.identityPlausible = diag_.chip.read && !diag_.chip.implausible();
    r.configWritten     = diag_.lastConfig.allWritesOk();
    r.configVerified    = diag_.lastConfig.allVerified();
    r.firstBadConfigReg = diag_.lastConfig.firstMismatch();

    ESP_LOGI(TAG, "  [%s] reset GPIO%d (%" PRIu32 " ms thap, %" PRIu32 " ms cho)",
             r.resetDone ? "PASS" : "FAIL", static_cast<int>(wiring_.resetPin),
             behavior_.reset.assertMs, behavior_.reset.settleMs);
    ESP_LOGI(TAG, "  [%s] doc identity  (khong bat buoc)",
             r.identityRead ? "PASS" : "bo qua");
    ESP_LOGI(TAG, "  [%s] ghi cau hinh%s", r.configWritten ? "PASS" : "FAIL",
             cfgErr == ESP_OK ? "" : " -- xem canh bao o tren");
    if (behavior_.chip.verifyWrites) {
        ESP_LOGI(TAG, "  [%s] doc lai cau hinh%s", r.configVerified ? "PASS" : "FAIL",
                 r.configVerified ? "" : " (thanh ghi dau tien sai o duoi)");
        if (!r.configVerified) {
            ESP_LOGW(TAG, "        thanh ghi 0x%02X", r.firstBadConfigReg);
        }
    }

    // With no finger on the glass the line must be idle. If it is not, either
    // the polarity is wrong or something is holding it -- and every interrupt
    // after this point would be noise.
    r.intIdleLevel   = gpio_get_level(wiring_.intPin);
    r.intIdleLevelOk = !intActive();
    ESP_LOGI(TAG, "  [%s] INT GPIO%d nghi o muc %d (mong doi %d)",
             r.intIdleLevelOk ? "PASS" : "FAIL", static_cast<int>(wiring_.intPin),
             r.intIdleLevel, wiring_.intActiveLow ? 1 : 0);
    if (!r.intIdleLevelOk) {
        ESP_LOGW(TAG, "        neu khong ai cham man hinh: kiem tra intActiveLow, "
                      "pull-up cua INT, hoac chip dang giu duong nay");
    }

    // The one check that needs no human at all: the chip pulses its own
    // interrupt line. Only available when the caller asked for it, because it
    // makes the chip interrupt continuously.
    if (behavior_.chip.irqSelfTest) {
        r.irqSelfPulseTried      = true;
        const uint32_t before    = irqCount_.load(std::memory_order_relaxed);
        vTaskDelay(pdMS_TO_TICKS(500));
        r.irqSelfPulseCount = irqCount_.load(std::memory_order_relaxed) - before;
        ESP_LOGI(TAG, "  [%s] EnTest: %" PRIu32 " xung trong 500 ms",
                 r.irqSelfPulseCount > 0 ? "PASS" : "FAIL", r.irqSelfPulseCount);
        if (r.irqSelfPulseCount == 0) {
            ESP_LOGE(TAG, "        chip khong keo duoc INT -> day noi hoac cau hinh "
                          "0xFA sai, KHONG phai loi phan mem doc frame");
        }
    }

    if (waitForTouchMs > 0) {
        ESP_LOGI(TAG, "  cham vao man hinh trong %" PRIu32 " ms...", waitForTouchMs);
        const uint32_t validBefore = diag_.validFrames;
        const uint32_t deadline    = nowMs() + waitForTouchMs;

        while (static_cast<int32_t>(nowMs() - deadline) < 0) {
            // Poll the interrupt counter rather than blocking on a notification:
            // this is already the task the ISR notifies.
            serviceInterrupts(nowMs());
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        r.framesObserved = diag_.validFrames - validBefore;
        r.touchObserved  = r.framesObserved > 0;
        ESP_LOGI(TAG, "  [%s] nhan duoc %" PRIu32 " frame hop le",
                 r.touchObserved ? "PASS" : "FAIL", r.framesObserved);
    }

    selfTestResult_ = r;
    logDiagnostics("sau self test");
}

}  // namespace touch
