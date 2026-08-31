#include "app_host.hpp"

#include <cinttypes>
#include <cstring>

#include "esp_log.h"

namespace ui {
namespace {
constexpr const char* TAG = "apphost";
}

int8_t AppRegistryBase::indexOf(const char* id) const
{
    if (id == nullptr) {
        return -1;
    }
    for (uint8_t i = 0; i < count(); ++i) {
        if (at(i).id != nullptr && std::strcmp(at(i).id, id) == 0) {
            return static_cast<int8_t>(i);
        }
    }
    return -1;
}

uint8_t registerDaemons(AppRegistryBase& registry, ::background::DaemonHost& host)
{
    uint8_t accepted = 0;
    for (uint8_t i = 0; i < registry.count(); ++i) {
        ::background::IAppDaemon* d = registry.at(i).daemon;
        if (d == nullptr) {
            continue;   // a UI-only app, which is most of them
        }
        if (!host.add(d)) {
            // host.add() has already said why. Logging the app's id as well is
            // what connects that message to a folder someone can open.
            ESP_LOGE(TAG, "app '%s': daemon khong dang ky duoc", registry.at(i).id);
            continue;
        }
        ++accepted;
    }
    return accepted;
}

// ---------------------------------------------------------------------------

void AppHost::attach(AppRegistryBase* registry, IApp* launcher,
                     const AppHostConfig& cfg)
{
    registry_ = registry;
    launcher_ = launcher;
    cfg_      = cfg;
}

bool AppHost::start()
{
    if (registry_ == nullptr || launcher_ == nullptr) {
        ESP_LOGE(TAG, "attach() chua duoc goi");
        return false;
    }
    nav_.reset();
    return activate(launcher_, ExitReason::Navigated);
}

IApp* AppHost::appAt(int8_t index) const
{
    if (index == kLauncherIndex) {
        return launcher_;
    }
    if (registry_ == nullptr || index < 0 || index >= static_cast<int8_t>(registry_->count())) {
        return nullptr;
    }
    return registry_->at(static_cast<uint8_t>(index)).ui;
}

const char* AppHost::currentId() const
{
    return idAt(nav_.current());
}

// ------------------------------------------------------------------ navigate

bool AppHost::showApp(uint8_t index)
{
    assertUiContext("AppHost::showApp");
    if (registry_ == nullptr || index >= registry_->count()) {
        ESP_LOGW(TAG, "khong co app so %u", index);
        return false;
    }

    // The stack decides whether this request is even worth obeying -- a repeat
    // of the current app, or a full stack. nav_stack.hpp explains why refusing
    // beats obeying for both.
    if (!nav_.push(static_cast<int8_t>(index))) {
        return false;
    }

    IApp* next = registry_->at(index).ui;
    if (!activate(next, ExitReason::Navigated)) {
        nav_.pop();  // put the user back where they actually still are
        return false;
    }
    return true;
}

bool AppHost::goBack()
{
    assertUiContext("AppHost::goBack");
    if (!nav_.pop()) {
        return false;  // already at the launcher; on a watch that is a no-op
    }
    IApp* next = appAt(nav_.current());
    if (next == nullptr) {
        return false;
    }
    return activate(next, ExitReason::Navigated);
}

bool AppHost::goHome()
{
    assertUiContext("AppHost::goHome");
    if (nav_.atLauncher()) {
        return false;
    }
    nav_.reset();
    return activate(launcher_, ExitReason::Navigated);
}

// -------------------------------------------------------------- accounting

const AppMemStats& AppHost::statsFor(int8_t index) const
{
    static const AppMemStats kEmpty{};
    if (index == kLauncherIndex) {
        return stats_[kMaxTrackedApps];
    }
    if (index < 0 || index >= static_cast<int8_t>(kMaxTrackedApps)) {
        return kEmpty;
    }
    return stats_[index];
}

AppMemStats* AppHost::mutableStatsFor(int8_t index)
{
    if (index == kLauncherIndex) {
        return &stats_[kMaxTrackedApps];
    }
    if (index < 0 || index >= static_cast<int8_t>(kMaxTrackedApps)) {
        return nullptr;  // registered beyond what we track; measured, not billed
    }
    return &stats_[index];
}

uint32_t AppHost::quotaFor(int8_t index) const
{
    if (index == kLauncherIndex || registry_ == nullptr || index < 0
        || index >= static_cast<int8_t>(registry_->count())) {
        return cfg_.defaultAppQuotaBytes;
    }
    const uint32_t own = registry_->at(static_cast<uint8_t>(index)).maxLvglBytes;
    return (own > 0) ? own : cfg_.defaultAppQuotaBytes;
}

const char* AppHost::idAt(int8_t index) const
{
    if (index == kLauncherIndex) {
        return "launcher";
    }
    if (registry_ == nullptr || index < 0
        || index >= static_cast<int8_t>(registry_->count())) {
        return "?";
    }
    return registry_->at(static_cast<uint8_t>(index)).id;
}

// ------------------------------------------------------------------ activate

bool AppHost::activate(IApp* app, ExitReason reasonForOutgoing)
{
    if (app == nullptr) {
        return false;
    }

    assertUiContext("AppHost::activate");

    // nav_ has already been moved by the caller, so this is the app we are
    // about to show. The launcher is exempt from every gate below: refusing it
    // would leave nothing on screen at all, which is a worse outcome than any
    // of the things the gates protect against.
    const int8_t targetIdx = nav_.current();
    const char*  targetId  = idAt(targetIdx);

    if (targetIdx != kLauncherIndex) {
        const AppMemStats* st = mutableStatsFor(targetIdx);
        if (st != nullptr && st->lockedOut) {
            ++refusedForMemory_;
            ESP_LOGE(TAG,
                     "tu choi mo '%s': lan truoc no ton %" PRIu32 " byte, qua "
                     "tran cung %" PRIu32 ". Cac app khac van mo binh thuong.",
                     targetId, st->peakBytes, cfg_.hardAppCeilingBytes);
            return false;
        }
        // The other kind of lock-out, and the only containment available for a
        // bug the hardware will not contain: this app was on screen the last
        // three times the board died. crash_log.hpp.
        if (forensics::quarantined(targetId)) {
            ESP_LOGE(TAG,
                     "tu choi mo '%s': da lam board chet %u lan lien tiep. "
                     "Dong ho van dung duoc, cac app khac khong bi anh huong.",
                     targetId,
                     static_cast<unsigned>(forensics::strikes(targetId)));
            return false;
        }
    }

    // RULE 9: check first, build second. Checking halfway through leaves a
    // half-built object tree and an app that has already been told it is
    // running.
    lv_mem_monitor_t before{};
    lv_mem_monitor(&before);
    if (before.free_size < cfg_.minLvglFreeToOpen) {
        ++refusedForMemory_;
        ESP_LOGE(TAG,
                 "tu choi mo app: LVGL heap con %" PRIu32 " byte, can it nhat %" PRIu32
                 ". Nguoi dung o nguyen man hinh hien tai.",
                 static_cast<uint32_t>(before.free_size), cfg_.minLvglFreeToOpen);
        return false;
    }

    // A screen is just an object with no parent. The HOST creates it and the
    // HOST deletes it -- rule 3. An app that deletes its own root pulls the
    // floor out from under this function.
    lv_obj_t* screen = lv_obj_create(nullptr);
    if (screen == nullptr) {
        ++refusedForMemory_;
        ESP_LOGE(TAG, "khong tao duoc screen");
        return false;
    }

    app->onCreate(screen);
    app->onEnter();

    // MEASURE HERE, before the outgoing app is torn down.
    //
    // Taking it after the teardown gives the NET change, which is not what rule
    // 10 asks for and is actively misleading: the first run reported the wrist
    // app as "+0 byte" purely because the launcher it replaced freed more than
    // it allocated. An app's budget is what IT costs, not what it costs minus
    // whatever happened to be on screen before it.
    lv_mem_monitor_t built{};
    lv_mem_monitor(&built);
    const uint32_t used = (before.free_size > built.free_size)
                              ? static_cast<uint32_t>(before.free_size - built.free_size)
                              : 0u;

    AppMemStats* st = mutableStatsFor(targetIdx);
    if (st != nullptr) {
        ++st->opens;
        st->lastBytes = used;
        if (used > st->peakBytes) {
            st->peakBytes = used;
        }
        const uint32_t quota = quotaFor(targetIdx);
        if (used > quota && targetIdx != kLauncherIndex) {
            ++st->overBudget;
            ESP_LOGE(TAG,
                     "'%s' vuot han muc: %" PRIu32 " byte / %" PRIu32
                     " byte cho phep (+%" PRIu32 ")",
                     targetId, used, quota, used - quota);
            // Already built, already entered -- it is on screen this time
            // whatever we think of it. What we CAN do is make sure it does not
            // get a second chance to take the pool down with it.
            if (used > cfg_.hardAppCeilingBytes) {
                st->lockedOut = true;
                ESP_LOGE(TAG,
                         "'%s' vuot ca tran cung %" PRIu32
                         " byte -> KHOA: lan sau se bi tu choi mo.",
                         targetId, cfg_.hardAppCeilingBytes);
            }
        }
    }

    // Only now is the outgoing app told it is leaving. Doing it earlier would
    // mean a failure above left it stopped but still on screen.
    IApp*     outgoing       = currentApp_;
    lv_obj_t* outgoingScreen = currentScreen_;
    if (outgoing != nullptr) {
        outgoing->onExit(reasonForOutgoing);
    }

    // Instant swap, deliberately: a slide transition redraws the whole 240x280
    // panel every frame, which is 4 MB/s at 30 fps against a 5 MB/s bus. See
    // display.md section 3.1 -- partial refresh is the operating condition
    // here, not an optimisation.
    lv_screen_load(screen);

    currentApp_    = app;
    currentScreen_ = screen;

    if (outgoing != nullptr) {
        outgoing->onDestroy();   // rule 3: forget every cached child pointer
    }
    if (outgoingScreen != nullptr) {
        lv_obj_delete(outgoingScreen);
    }

    lv_mem_monitor_t after{};
    lv_mem_monitor(&after);
    if (used > peakAppBytes_) {
        peakAppBytes_ = used;
    }

    const int8_t outgoingIdx = activeIndex_;

    // The outgoing app reached onExit() and onDestroy() without taking the
    // board down with it. That is what "ran clean" means, and it is what clears
    // a crash record -- so one bad afternoon does not condemn an app forever.
    if (outgoing != nullptr && outgoingIdx != kLauncherIndex) {
        forensics::clearStrikes(idAt(outgoingIdx));
    }

    // LEAK ACCOUNTING.
    //
    // Only the launcher gives a clean reading, and it gives an exact one. It is
    // the one screen returned to repeatedly, and by the time this line runs the
    // outgoing app has been fully destroyed -- so free memory here should be
    // the SAME NUMBER every single time. It is a fixed screen with a fixed
    // cost; nothing else is resident.
    //
    // Any drift is memory an app allocated and did not hand back: a style never
    // freed, an lv_timer never deleted, an object parented to the screen layer
    // instead of to the app's root. None of those show up as a crash, and none
    // of them are visible on the screen that caused them. They surface weeks
    // later as a watch that has stopped being able to open anything.
    if (targetIdx == kLauncherIndex) {
        const uint32_t freeNow = static_cast<uint32_t>(after.free_size);
        if (!haveBaseline_) {
            baselineFree_     = freeNow;
            lastLauncherFree_ = freeNow;
            haveBaseline_     = true;
        } else {
            // Charged to whoever we just came back from.
            //
            // Exact for launcher -> app -> launcher, which is the whole of
            // normal use on a watch. For launcher -> A -> B -> launcher the
            // delta covers both and lands entirely on B, so the ATTRIBUTION is
            // approximate while the TOTAL below stays exact. Worth knowing
            // before chasing a number: the total says whether there is a leak,
            // the per-app column says where to look first.
            const int32_t sinceLast = static_cast<int32_t>(lastLauncherFree_)
                                      - static_cast<int32_t>(freeNow);
            AppMemStats* out = mutableStatsFor(outgoingIdx);
            if (out != nullptr && outgoingIdx != kLauncherIndex) {
                out->residueBytes += sinceLast;
            }
            lastLauncherFree_ = freeNow;
            leaked_           = static_cast<int32_t>(baselineFree_)
                      - static_cast<int32_t>(freeNow);
        }
    }

    activeIndex_ = targetIdx;

    // Written to RTC memory, which survives a reset. If this app takes the
    // board down, the next boot opens with its name instead of a mystery.
    forensics::setCurrentApp(targetId);

    ++transitions_;

    // RULE 10: every app has a budget, and the only way to know whether it is
    // over is to print the number on every transition.
    ESP_LOGI(TAG, "-> %s  (LVGL +%" PRIu32 " byte, con %" PRIu32 ")", currentId(),
             used, static_cast<uint32_t>(after.free_size));
    return true;
}

void AppHost::tick(uint32_t nowMs)
{
    // Only the foreground app. A background app runs nothing -- rule 4, and the
    // reason a watch can still sleep with fifteen apps installed.
    if (currentApp_ != nullptr) {
        currentApp_->onTick(nowMs);
    }
}

void AppHost::logDiagnostics(const char* where) const
{
    ESP_LOGI(TAG,
             "[%s] dang o '%s'  sau=%u  chuyen=%" PRIu32 "  tu-choi-bo-nho=%" PRIu32
             "  app ton nhieu nhat=%" PRIu32 " byte",
             where, currentId(), nav_.depth(), transitions_, refusedForMemory_,
             peakAppBytes_);

    if (nav_.rejectedRepeat() != 0 || nav_.rejectedFull() != 0) {
        ESP_LOGI(TAG, "  dieu huong bi bo: trung=%" PRIu32 " day-stack=%" PRIu32,
                 nav_.rejectedRepeat(), nav_.rejectedFull());
    }

    // The per-app ledger. Only apps that have actually been opened appear:
    // a table of zeroes teaches whoever reads this log to skip the block.
    if (registry_ != nullptr) {
        bool printedHeader = false;
        for (uint8_t i = 0; i < registry_->count() && i < kMaxTrackedApps; ++i) {
            const AppMemStats& st = stats_[i];
            if (st.opens == 0) {
                continue;
            }
            if (!printedHeader) {
                ESP_LOGI(TAG, "  so bo nho tung app (pool LVGL 64 KB):");
                printedHeader = true;
            }
            const char* id = registry_->at(i).id;
            ESP_LOGI(TAG,
                     "    %-10s mo=%" PRIu32 "  lan cuoi=%" PRIu32
                     " byte  dinh=%" PRIu32 "/%" PRIu32 "  con no=%" PRId32
                     " byte%s",
                     id, st.opens, st.lastBytes, st.peakBytes,
                     quotaFor(static_cast<int8_t>(i)), st.residueBytes,
                     st.lockedOut ? "  [DA KHOA]" : "");

            // Loud, because this is the number that decides whether the watch
            // survives a week. 256 bytes of slack absorbs the allocator's own
            // rounding without hiding a real leak: a leak repeats, and the
            // total climbs every time the app is visited.
            if (st.residueBytes > 256) {
                ESP_LOGW(TAG,
                         "    '%s' RO RI: %" PRId32 " byte khong tra lai sau %"
                         PRIu32 " lan mo (~%" PRId32 " byte/lan). Tim style, "
                         "lv_timer hoac object khong gan vao root cua app.",
                         id, st.residueBytes, st.opens,
                         st.residueBytes / static_cast<int32_t>(st.opens));
            }
            if (st.overBudget > 0) {
                ESP_LOGW(TAG, "    '%s' vuot han muc %" PRIu32 " lan", id,
                         st.overBudget);
            }
        }

        if (haveBaseline_ && leaked_ > 256) {
            ESP_LOGW(TAG,
                     "  pool LVGL da mat %" PRId32 " byte so voi luc dau "
                     "(%" PRIu32 " -> %" PRIu32 "). Cong don cua moi app o tren.",
                     leaked_, baselineFree_, lastLauncherFree_);
        }
    }

    if (forensics::culprit()[0] != 0) {
        ESP_LOGW(TAG, "  '%s' dang bi ghi so: %u lan lam board chet lien tiep",
                 forensics::culprit(),
                 static_cast<unsigned>(forensics::strikes(forensics::culprit())));
    }
}

}  // namespace ui
