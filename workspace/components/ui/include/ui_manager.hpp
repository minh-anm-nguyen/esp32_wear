// The LVGL runtime: the one task allowed to touch LVGL, and the glue that
// connects it to components/display and components/touch.
//
// THE RULE THIS CLASS EXISTS TO ENFORCE
//
// Exactly one task owns the LVGL object tree. CONFIG_LV_OS_NONE=y makes
// lv_lock()/lv_unlock() no-ops, so there is NO safety net: a stray LVGL call
// from the IMU or touch task will not deadlock and will not assert, it will
// corrupt the tree and surface later as a random reboot.
//
// Every other task talks to the UI through post(). That is the whole API.
//
// See doc-design/ui.md, and doc-design/LVGL-UI-technical-challenges.md for the
// seventeen board-specific traps this integration has to get past.
#pragma once

#include "app.hpp"
#include "app_host.hpp"
#include "app_registry.hpp"
#include "flush_coordinator.hpp"
#include "launcher.hpp"
#include "ui_context.hpp"
#include "ui_command.hpp"

#include "display_manager.hpp"
#include "touch_manager.hpp"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <atomic>

#include "lvgl.h"

namespace ui {

struct UiConfig {
    // 6: below touch (11), IMU (10) and button (9), above the buzzer (5).
    // Fixed by doc-design/i2c-bus-design.md section 11; the UI touches SPI, not
    // I2C, so it stays out of that priority-inversion problem entirely.
    UBaseType_t taskPriority{6};

    // LVGL needs far more stack than the other tasks. A starting point: measure
    // with uxTaskGetStackHighWaterMark() against the heaviest screen and tighten.
    uint32_t taskStackSize{8192};

    // Matches display::Config::transferTimeoutMs. A 40-line flush is 3.84 ms of
    // work, so 200 ms means the panel or the DMA channel is dead, not slow.
    uint32_t flushTimeoutMs{200};

    // Ceiling on how long the task sleeps when LVGL says it has nothing to do.
    uint32_t maxIdleMs{1000};

    // Sleep ceiling WHILE A FINGER IS DOWN. Input is sampled this often even
    // though the screen still refreshes at LV_DEF_REFR_PERIOD, so the position
    // being drawn is a few milliseconds stale rather than a whole frame stale.
    // 6 ms is comfortably below the CST816T's own ~12 ms interrupt interval, so
    // no frame from the chip waits for us.
    uint32_t touchPollMs{6};

    // Repaint the instant a new finger position arrives, instead of waiting for
    // the refresh timer.
    //
    // After the ordering fix this wait was the largest term left: a point
    // sampled 1 ms ago still sat until the timer came due. The chip only
    // produces a position every ~12 ms, so painting on arrival converges on the
    // chip's own rate rather than beating uselessly against it.
    bool refreshOnTouchMove{true};

    // Refresh period WHILE A FINGER IS DOWN, overriding LV_DEF_REFR_PERIOD.
    //
    // Fresh input is only half of touch latency; the other half is how long the
    // fresh position waits to be painted. 16 ms doubles the paint rate to ~60 Hz
    // for the seconds a finger is actually on the glass, and costs nothing the
    // rest of the time -- which is almost always, on a watch.
    //
    // The bus can afford it: a moving marker dirties two small regions, so at
    // 60 Hz that is roughly 120 flushes/s x 3.84 ms, about 46% of the SPI
    // budget. A FULL-screen animation at this rate would not fit -- see
    // display.md section 3.1.
    uint32_t touchRefreshMs{16};

    uint8_t controlQueueLen{8};
    uint8_t criticalQueueLen{4};  // separate, so navigation cannot crowd out an alarm
};

class UiManager {
public:
    UiManager()                            = default;
    ~UiManager();
    UiManager(const UiManager&)            = delete;
    UiManager& operator=(const UiManager&) = delete;

    // `touch` may be null: a board whose touch panel failed to come up should
    // still show a screen.
    //
    // `registry` must outlive the runtime and must be fully populated: apps are
    // registered at composition time, before the UI task exists, so the list is
    // read-only from then on and needs no locking.
    esp_err_t init(display::DisplayManager& disp, touch::TouchManager* touch,
                   AppRegistryBase& registry, const UiConfig& cfg = {});

    // Creates the UI task. Everything LVGL happens there from this point on.
    esp_err_t start();

    // Thread-safe. THE only way another task may affect the UI.
    PostResult post(const UiCommand& cmd);
    PostResult postFromIsr(const UiCommand& cmd, BaseType_t* higherWoken);

    RuntimeState            state() const { return state_.load(); }
    const FlushCoordinator& flush() const { return flush_; }
    const AppHost&          apps() const { return host_; }
    uint32_t                frames() const { return frames_; }
    uint32_t                droppedCommands() const { return dropped_; }

    // Stack headroom of the UI task, in bytes. 0 before start().
    uint32_t taskStackHeadroom() const;

    void logDiagnostics(const char* where) const;

private:
    // ---- LVGL trampolines. All run in the UI task except onTransferDone. ----
    static void flushCb(lv_display_t* disp, const lv_area_t* area, uint8_t* px);
    static void flushWaitCb(lv_display_t* disp);
    static void touchReadCb(lv_indev_t* indev, lv_indev_data_t* data);
    static bool onTransferDone(void* ctx);   // ISR CONTEXT
    static void taskTrampoline(void* arg);

    esp_err_t initLvgl();
    esp_err_t initDisplay();
    esp_err_t initInput();
    static void onLauncherPick(void* ctx, uint8_t appIndex);

    void      taskBody();
    void      drainCommands();
    void      applyCommand(const UiCommand& cmd);

    display::DisplayManager* display_{nullptr};
    touch::TouchManager*     touch_{nullptr};
    UiConfig                 cfg_{};

    lv_display_t* lvDisplay_{nullptr};
    lv_indev_t*   lvIndev_{nullptr};
    void*         buf1_{nullptr};
    void*         buf2_{nullptr};

    // The launcher and the host that runs apps. The runtime owns them; it does
    // not own any app -- those live in components/apps and are registered by
    // the composition root.
    Launcher launcher_{};
    AppHost  host_{};

    FlushCoordinator flush_{};

    QueueHandle_t controlQueue_{nullptr};
    QueueHandle_t criticalQueue_{nullptr};
    TaskHandle_t  task_{nullptr};

    std::atomic<RuntimeState> state_{RuntimeState::Uninitialized};
    std::atomic<uint32_t>     sequence_{0};

    // Touch state carried between read_cb calls. LVGL decides which widget was
    // clicked from the coordinate reported WITH the release, so the last valid
    // point has to survive the finger leaving the glass.
    lv_point_t lastPoint_{0, 0};
    bool       pressed_{false};

    // Whether the refresh timer is currently in its faster, finger-down period.
    // Tracked so the period is only written when it actually changes, rather
    // than every pass.
    bool fastRefresh_{false};

    // Set by touchReadCb when a transition was actually consumed this pass, so
    // the loop knows a repaint would show something new rather than redrawing
    // the same pixels.
    bool pointChanged_{false};

    // How stale a touch sample was by the time the UI consumed it, in ms --
    // measured from the timestamp the touch task stamped on it. Isolates the
    // touch-task-to-UI-task half of the latency from the render half, so the
    // next round of tuning is arithmetic rather than guesswork.
    uint32_t inputAgeMaxMs_{0};
    uint32_t inputAgeSumMs_{0};
    uint32_t inputAgeCount_{0};
    uint32_t forcedRefreshes_{0};

    // LVGL heap figures, sampled BY THE UI TASK each pass and read from
    // anywhere. lv_mem_monitor() walks LVGL's allocator, so calling it from the
    // diagnostics loop -- as an earlier version of logDiagnostics() did -- was
    // an LVGL call from the wrong task, exactly what ui_context.hpp exists to
    // catch. Caching costs two words and removes the hazard entirely.
    uint32_t lvglFreeBytes_{0};
    uint32_t lvglLargestBytes_{0};

    uint32_t frames_{0};
    uint32_t dropped_{0};

};

}  // namespace ui
