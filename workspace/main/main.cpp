// Composition root and product policy. Nothing else.
//
// WHAT MOVED OUT, AND WHY
//
// This file used to be 530 lines doing three jobs: wiring the hardware, running
// a loop, and deciding product behaviour. Only the first belongs in one place;
// the other two are what made it grow with every feature and what would have
// made it the file every future app has to edit.
//
//   board.hpp     every driver on this board, and the order they come up in
//   services.hpp  the domain layer: driver output -> facts an app can read
//   main.cpp      what this product DOES with them
//
// See doc-design/app-architecture.md sections 5 and 7.
#include "board.hpp"
#include "crash_log.hpp"
#include "daemon_host.hpp"
#include "services.hpp"
#include "ui_manager.hpp"

#include "diag_app.hpp"
#include "touch_app.hpp"
#include "wrist_app.hpp"
#include "wrist_daemon.hpp"

#include <cinttypes>
#include <cstdio>

#include "esp_heap_caps.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "app";

// ---------------------------------------------------------------- log helpers

const char* eventName(button::ButtonEvent e)
{
    switch (e) {
    case button::ButtonEvent::PRESS_DOWN:         return "PRESS_DOWN";
    case button::ButtonEvent::CLICK:              return "CLICK";
    case button::ButtonEvent::DOUBLE_CLICK:       return "DOUBLE_CLICK";
    case button::ButtonEvent::LONG_PRESS:         return "LONG_PRESS";
    case button::ButtonEvent::LONG_PRESS_RELEASE: return "LONG_PRESS_RELEASE";
    case button::ButtonEvent::DOUBLE_CLICK_HOLD:  return "DOUBLE_CLICK_HOLD";
    default:                                      return "?";
    }
}

const char* imuEventName(imu::ImuEventMsg::Type t)
{
    switch (t) {
    case imu::ImuEventMsg::Type::WAKE_ON_MOTION: return "WAKE_ON_MOTION";
    case imu::ImuEventMsg::Type::STEP:           return "STEP";
    case imu::ImuEventMsg::Type::TAP:            return "TAP";
    case imu::ImuEventMsg::Type::ANY_MOTION:     return "ANY_MOTION";
    case imu::ImuEventMsg::Type::NO_MOTION:      return "NO_MOTION";
    case imu::ImuEventMsg::Type::SIG_MOTION:     return "SIG_MOTION";
    case imu::ImuEventMsg::Type::BUS_ERROR:      return "BUS_ERROR";
    }
    return "?";
}

// -------------------------------------------------------------- product policy
//
// Everything below this line is a decision about how the WATCH should behave,
// not about how the hardware works. It is the part that will migrate into apps
// once there is an app framework to migrate it into; keeping it in one labelled
// block is what makes that migration a move rather than an excavation.

const buzzer::Pattern* soundFor(button::ButtonEvent e)
{
    switch (e) {
    // The only sound tied to the pin instead of to a recognised gesture, and so
    // the only one that can be instant: it goes out about 20 ms after the finger
    // lands, which is what makes the button feel responsive.
    case button::ButtonEvent::PRESS_DOWN:   return &buzzer::patterns::kClick;

    // Deliberately silent. PRESS_DOWN already acknowledged this same press some
    // 250 ms earlier; beeping again when the double click window finally expires
    // would just sound like an echo of it.
    case button::ButtonEvent::CLICK:        return nullptr;

    case button::ButtonEvent::DOUBLE_CLICK: return &buzzer::patterns::kOk;

    // kAlarm has repeat = 0 and priority 200. It loops until something stops it,
    // and while it runs a CLICK (priority 10) cannot cut in. Try it: keep
    // clicking during the alarm and nothing happens to the sound.
    case button::ButtonEvent::LONG_PRESS:   return &buzzer::patterns::kAlarm;

    default:                                return nullptr;
    }
}

// Runs in the IMU task, via WristService. Product policy: a raise is worth a
// click, a lower is not.
//
// The EVENT is used here rather than the topic, and that is the distinction
// topic.hpp is built around: waking the screen is a thing that HAPPENED and
// must not be coalesced away. What the screen then DISPLAYS comes from the
// topic. Both exist because they answer different questions.
void onWristEvent(void* ctx, motion::MotionEvent ev, uint32_t nowMs)
{
    auto* buz = static_cast<buzzer::BuzzerManager*>(ctx);
    if (ev == motion::MotionEvent::WRIST_RAISE) {
        ESP_LOGI(TAG, "NANG CO TAY (t=%" PRIu32 " ms) -> bat man hinh", nowMs);
        buz->play(buzzer::patterns::kClick);
    } else if (ev == motion::MotionEvent::WRIST_LOWER) {
        ESP_LOGI(TAG, "ha tay (t=%" PRIu32 " ms) -> tat man hinh", nowMs);
    }
}

// Feeds the diagnostics app. Runs in the UI task, called by the app itself.
//
// A snapshot rather than a UiManager pointer: an app must be able to READ the
// runtime's health without being able to reach in and change it.
void fillDiagSnapshot(void* ctx, apps::DiagSnapshot& out)
{
    auto* ui = static_cast<ui::UiManager*>(ctx);

    out.frames        = ui->frames();
    out.flushPerFrame100 =
        (ui->frames() != 0) ? (ui->flush().started() * 100u / ui->frames()) : 0u;
    out.flushBalanced   = ui->flush().balanced();
    out.stackFreeBytes  = ui->taskStackHeadroom();

    lv_mem_monitor_t m{};
    lv_mem_monitor(&m);
    out.lvglFreeBytes    = static_cast<uint32_t>(m.free_size);
    out.lvglLargestBytes = static_cast<uint32_t>(m.free_biggest_size);

    out.sramFreeBytes =
        static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    out.sramLargestBytes =
        static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

// ------------------------------------------------------------------- runtime

void logHelp(const board::Board& b)
{
    const button::ButtonBehavior& bhow = b.buttonBehavior();

    ESP_LOGI(TAG, "san sang. Nut GPIO%d, coi GPIO%d",
             static_cast<int>(board::kButtonPin),
             static_cast<int>(board::kBuzzerPin));
    ESP_LOGI(TAG, "  cham xuong   -> bip ngan NGAY (~20ms, chi qua debounce)");
    ESP_LOGI(TAG, "  double click -> hai tieng len giong");
    ESP_LOGI(TAG, "  giu %" PRIu32 "ms   -> bao thuc lap vo han (click luc nay bi bo qua)",
             bhow.longPressMs);
    ESP_LOGI(TAG, "  nha tay      -> tat bao thuc");
    ESP_LOGI(TAG, "luu y: CLICK van bi tre %" PRIu32 " ms vi FSM phai loai tru double "
                  "click; no khong keu, tieng bip da phat luc cham xuong roi",
             bhow.doubleClickMs);

    if (b.imuReady()) {
        ESP_LOGI(TAG, "nang co tay len ngang mat -> se keu mot tieng bip");
    }
}

// The bring-up loop. Drains every queue and prints what it finds.
//
// This is what components/ui will replace: once the UI task exists it becomes
// the reader of these queues and of the wrist topic, and app_main goes back to
// doing nothing at all. Until then this loop is the only consumer, and it is
// the reason the touch and IMU queues do not silently overflow.
void runDiagnosticLoop(board::Board& b, app::Services& services, ui::UiManager& ui,
                       background::DaemonHost& d)
{
    button::ButtonEventMsg msg;
    uint32_t               lastTouchDiagMs = 0;

    uint32_t lastUiDiagMs = 0;

    // DRAIN EVERY QUEUE ON EVERY PASS, and never block on just one of them.
    // Waiting forever on the button queue once let the IMU queue fill up and
    // start dropping -- which is exactly what the "queue day, bo su kien 0"
    // warnings in the log were saying. The button wait below carries a 100 ms
    // timeout for that reason, not for responsiveness.
    for (;;) {
        // The wrist topic is NOT read here any more: the UI task reads it, in
        // the UI task, through wristStatusLine(). Two readers would be legal --
        // each owns its own cursor -- but one of them putting it on the screen
        // is the point of the exercise.

        // --- raw driver queues ---
        imu::ImuEventMsg iev;
        while (b.imu().waitEvent(iev, 0)) {
            ESP_LOGI(TAG, "IMU  %-15s  t=%" PRIu32 " ms", imuEventName(iev.type),
                     iev.timestampMs);
        }

        // THE TOUCH QUEUE IS NOT DRAINED HERE. It has exactly one consumer, and
        // that consumer is the LVGL input callback in the UI task.
        //
        // This loop used to pop it too, "just for logging", and that was a real
        // bug with a very confusing symptom: a vertical swipe off the edge of
        // the screen would sometimes leave the touch stuck down, and only the
        // NEXT touch would release it.
        //
        // popTransition() CONSUMES. Two consumers race, and whoever wins steals
        // the event. When this loop won the Up, LVGL never saw it and stayed at
        // LV_INDEV_STATE_PRESSED forever. It showed up on fast swipes because
        // this loop drains the WHOLE queue every ~100 ms while the UI task polls
        // at ~31 fps, so a burst of Moves followed quickly by an Up was very
        // likely to be scooped up here in one sweep.
        //
        // Nothing is lost by removing it: touch::Behavior::logFrames already
        // prints every frame as raw hex plus `raw(x,y) -> (x,y)` from the touch
        // task, which is what the orientation bring-up actually needs.

        // Counters, on a slow timer. The per-frame detail is in the raw log
        // above; this is the summary that answers "is the release timeout
        // right" and "is anything being dropped".
        const uint32_t nowTick = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (b.touch().isRunning() && nowTick - lastTouchDiagMs >= 10000) {
            lastTouchDiagMs = nowTick;
            b.touch().logDiagnostics("dinh ky 10 s");
        }
        if (nowTick - lastUiDiagMs >= 10000) {
            lastUiDiagMs = nowTick;
            ui.logDiagnostics("dinh ky 10 s");
            // Rides the same timer but prints nothing while the bus is clean,
            // so it costs no log space until it has something to say.
            b.logBusHealth("dinh ky 10 s");
            d.logDiagnostics("dinh ky 10 s");
        }

        if (!b.button().waitEvent(msg, 100)) {
            continue;  // just a timeout: go back and check the other queues
        }

        // timestampMs      = when the FSM reached its conclusion
        // pressTimestampMs = when the user started pressing
        // The difference between them is the latency of that event type.
        ESP_LOGI(TAG, "GPIO%-2d  %-19s  press=%" PRIu32 " ms  phat=%" PRIu32 " ms  (tre %" PRIu32 " ms)",
                 static_cast<int>(msg.pin), eventName(msg.event),
                 msg.pressTimestampMs, msg.timestampMs,
                 msg.timestampMs - msg.pressTimestampMs);

        // The alarm never ends on its own, so releasing the button is what stops
        // it. silence() outranks nothing -- stop always wins.
        if (msg.event == button::ButtonEvent::LONG_PRESS_RELEASE) {
            b.buzzer().silence();
            continue;
        }

        // BACK IS THE BUTTON, not a swipe -- for now, and deliberately.
        //
        // touch::Geometry is still the identity and expected to be wrong, so an
        // edge-swipe gesture would be unreliable in a way that is hard to tell
        // apart from a framework bug. A physical button always works, so the
        // user can never be stranded inside an app even if every tap lands in
        // the mirror image of where it was aimed. Swipe-back is worth adding
        // once the orientation is settled.
        if (msg.event == button::ButtonEvent::CLICK) {
            ui.post({ui::UiCommandType::GO_BACK, 0, 0, 0});
        } else if (msg.event == button::ButtonEvent::DOUBLE_CLICK) {
            ui.post({ui::UiCommandType::GO_HOME, 0, 0, 0});
        }

        if (const buzzer::Pattern* sound = soundFor(msg.event)) {
            // Returns ESP_ERR_TIMEOUT if the command queue is full, which only
            // happens when someone presses far faster than a pattern can play.
            // Dropping a beep is the right answer there: this loop must not block.
            const esp_err_t err = b.buzzer().play(*sound);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "play() bo qua: %s", esp_err_to_name(err));
            }
        }
    }
}

}  // namespace

extern "C" void app_main(void)
{
    // Static, not stack: app_main's frame is small and these own every driver
    // task's context. They live until reboot by design -- see the note on
    // runtime restart in doc-design/app-architecture.md section 15.
    static board::Board   hw;
    static app::Services  services;
    static ui::UiManager  uiRuntime;
    static ui::AppRegistry<6> registry;

    // 0. What the PREVIOUS boot left behind. First, so that if bring-up itself
    //    is what keeps dying, the message naming the cause is already out
    //    before the thing that dies runs again.
    forensics::reportBoot();

    // 1. Hardware. Never ESP_ERROR_CHECK: aborting reboots the chip and the one
    //    useful error message scrolls past inside an endless boot loop.
    const esp_err_t herr = hw.initDevices();
    if (herr != ESP_OK) {
        ESP_LOGE(TAG, "khoi tao phan cung that bai: %s -- van chay tiep",
                 esp_err_to_name(herr));
    }

    // 2. Domain layer. MUST be between initDevices() and startSensors(): the
    //    fan-out is sealed by the latter. board.hpp explains why this is two
    //    calls instead of one.
    if (services.attach(hw) != ESP_OK) {
        ESP_LOGE(TAG, "gan service that bai -- du lieu cam bien se khong toi UI");
    }

    // 3. Product policy, wired before anything can fire.
    services.wrist().setEventCallback(onWristEvent, &hw.buzzer());

    // 4. Apps and their daemons.
    //
    //    CONSTRUCTED AND REGISTERED HERE, and the position is load-bearing in
    //    two ways that were both wrong in the first version:
    //
    //    a) BEFORE startSensors(). A daemon attaches itself to the IMU fan-out
    //       in onStart(), and startSensors() SEALS that fan-out. Registering
    //       afterwards would have every daemon silently fail to subscribe.
    //
    //    b) OUTSIDE the display check below. A watch whose screen failed to
    //       come up should still count activity -- the background half of an
    //       app has nothing to do with whether anyone can see it. Putting this
    //       inside the display branch tied the two together for no reason.
    //
    //    Apps are handed only what they need: WristApp gets two topics, the
    //    daemon gets the sample stream, DiagApp gets a snapshot function. None
    //    of them gets a Board, a Services, or a driver -- app.hpp rule 6,
    //    enforced by what is absent from these constructors.
    static apps::WristDaemon wristDaemon{};
    static apps::WristApp    wristApp{services.wrist().state(), wristDaemon.state()};
    static apps::TouchApp    touchApp;
    static apps::DiagApp     diagApp{fillDiagSnapshot, &uiRuntime};

    // Designated initialisers, and add() is [[nodiscard]] -- the first version
    // of these lines threw the result away while the header said not to, which
    // would have made a full registry lose an app in silence.
    const bool registered =
        registry.add({.id     = "wrist",
                      .title  = "Co tay",
                      .icon   = LV_SYMBOL_EYE_OPEN,
                      .ui     = &wristApp,
                      .daemon = &wristDaemon})
        && registry.add({.id    = "touch",
                         .title = "Cham",
                         .icon  = LV_SYMBOL_GPS,
                         .ui    = &touchApp})
        && registry.add({.id    = "diag",
                         .title = "Chan doan",
                         .icon  = LV_SYMBOL_SETTINGS,
                         .ui    = &diagApp});
    if (!registered) {
        ESP_LOGE(TAG, "registry day -- co app khong xuat hien trong launcher");
    }

    // 5. Background halves get their tasks now: while the fan-out still accepts
    //    subscribers, and long before any UI exists.
    //
    //    ONE subscriber goes into the fan-out no matter how many daemons there
    //    are -- the host itself. It copies each sample into one queue per
    //    daemon and returns. That is what keeps app code out of the IMU task
    //    and keeps the sensor's cost proportional to the NUMBER of daemons
    //    rather than to what any of them does.
    static background::DaemonHost daemons{};

    ESP_LOGI(TAG, "%u daemon dang ky", ui::registerDaemons(registry, daemons));

    if (!hw.imuSamples().add(&daemons)) {
        ESP_LOGE(TAG, "DaemonHost khong vao duoc luong IMU -- moi daemon se doi mai");
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(daemons.start());

    // 6. Sensors run. This seals the fan-out: no more subscribers after here.
    const esp_err_t serr = hw.startSensors();
    if (serr != ESP_OK) {
        ESP_LOGW(TAG, "cam bien khong day du: %s", esp_err_to_name(serr));
    }

    // 7. The UI runtime. Last, because it needs a live display and the topics
    //    it reads must already be publishing.
    //
    //    From here on the UI task is the ONLY task allowed to call LVGL.
    //    CONFIG_LV_OS_NONE=y means lv_lock() is a no-op, so a stray call from
    //    another task does not deadlock or assert -- it corrupts the object tree
    //    and reboots at random, much later. post() is the only way in, and
    //    ui::assertUiContext() is what says so out loud when it is violated.
    if (hw.displayReady()) {
        const esp_err_t uerr = uiRuntime.init(
            hw.display(), hw.touchReady() ? &hw.touch() : nullptr, registry);
        if (uerr == ESP_OK) {
            if (uiRuntime.start() != ESP_OK) {
                ESP_LOGE(TAG, "UI task khong chay duoc");
            }
        } else {
            ESP_LOGE(TAG, "UI khong khoi tao duoc: %s", esp_err_to_name(uerr));
        }
    } else {
        ESP_LOGW(TAG, "khong co man hinh -- bo qua UI, daemon van chay");
    }

    logHelp(hw);
    runDiagnosticLoop(hw, services, uiRuntime, daemons);
}
