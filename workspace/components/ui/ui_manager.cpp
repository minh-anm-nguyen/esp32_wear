#include "ui_manager.hpp"

#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace ui {
namespace {
constexpr const char* TAG = "ui";
}

UiManager::~UiManager()
{
    // No teardown path yet, by design: this firmware is "init once, live until
    // reboot" (app-architecture.md section 15, decision 4). Writing a deinit
    // that has never been exercised would be worse than not having one -- it
    // would look like a guarantee.
    if (task_ != nullptr) {
        ESP_LOGW(TAG, "UiManager bi huy trong khi task van chay");
    }
}

// ----------------------------------------------------------------------- init

esp_err_t UiManager::init(display::DisplayManager& disp, touch::TouchManager* touch,
                          AppRegistryBase& registry, const UiConfig& cfg)
{
    if (state_.load() != RuntimeState::Uninitialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!disp.isInitialized()) {
        ESP_LOGE(TAG, "display chua init -- khong the dung LVGL len tren");
        return ESP_ERR_INVALID_STATE;
    }

    display_ = &disp;
    touch_   = touch;
    cfg_     = cfg;
    state_.store(RuntimeState::Starting);

    controlQueue_  = xQueueCreate(cfg_.controlQueueLen, sizeof(UiCommand));
    criticalQueue_ = xQueueCreate(cfg_.criticalQueueLen, sizeof(UiCommand));
    if (controlQueue_ == nullptr || criticalQueue_ == nullptr) {
        state_.store(RuntimeState::Failed);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = initLvgl();
    if (err == ESP_OK) {
        err = initDisplay();
    }
    if (err == ESP_OK) {
        err = initInput();
    }
    if (err != ESP_OK) {
        state_.store(RuntimeState::Failed);
        return err;
    }

    // The launcher reports a pick; the host decides what to do with it. Neither
    // navigates from inside the LVGL event callback -- doing so would delete the
    // object tree LVGL is still dispatching an event on.
    launcher_.attach(&registry, &UiManager::onLauncherPick, this);
    host_.attach(&registry, &launcher_);

    if (!host_.start()) {
        ESP_LOGE(TAG, "khong dung duoc launcher");
        state_.store(RuntimeState::Failed);
        return ESP_FAIL;
    }
    return ESP_OK;
}

void UiManager::onLauncherPick(void* ctx, uint8_t appIndex)
{
    auto* self = static_cast<UiManager*>(ctx);

    // Posted, not called. This runs inside an LVGL event on a launcher tile;
    // opening the app here would tear down that very tile mid-dispatch. Going
    // through the queue means the swap happens at the top of the next frame,
    // with nothing else in flight.
    self->post({UiCommandType::SHOW_APP, appIndex, 0, 0});
}

esp_err_t UiManager::initLvgl()
{
    lv_init();

    // ONE tick source, and this is the only place it is set.
    //
    // lv_tick_set_cb() is the LVGL 9 mechanism; LV_TICK_CUSTOM is v8 and does
    // not exist here. Also calling lv_tick_inc() anywhere would make LVGL time
    // run at double speed with no compile error -- and "animations are twice as
    // fast" reads as a refresh-rate misconfiguration, not as a tick bug.
    //
    // esp_timer, NOT the RTC: wall-clock time can jump forwards or backwards on
    // a sync, and an animation timebase that jumps skips to its final frame.
    lv_tick_set_cb([]() -> uint32_t {
        return static_cast<uint32_t>(esp_timer_get_time() / 1000);
    });

    return ESP_OK;
}

esp_err_t UiManager::initDisplay()
{
    const display::Geometry& geo = display_->geometry();

    lvDisplay_ = lv_display_create(geo.width, geo.height);
    if (lvDisplay_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    lv_display_set_user_data(lvDisplay_, this);

    // The buffers come from DisplayManager, never from here: it is the only
    // place that can enforce MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL. An ordinary
    // heap buffer works for a while and then fails in a way nobody can
    // reproduce. display.md section 9.2.
    buf1_ = display_->allocFlushBuffer();
    buf2_ = display_->allocFlushBuffer();
    if (buf1_ == nullptr || buf2_ == nullptr) {
        ESP_LOGE(TAG, "khong cap phat duoc draw buffer (%u byte moi cai)",
                 static_cast<unsigned>(display_->flushBufferBytes()));
        return ESP_ERR_NO_MEM;
    }

    // buf_size is in BYTES in LVGL 9. It was in PIXELS in LVGL 8, so a copied
    // example is out by exactly a factor of two for RGB565 -- either overrunning
    // the buffer or flushing half the lines it should.
    lv_display_set_buffers(lvDisplay_, buf1_, buf2_,
                           static_cast<uint32_t>(display_->flushBufferBytes()),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_display_set_flush_cb(lvDisplay_, &UiManager::flushCb);

    // NOT optional. Without it lv_refr.c does `while(disp->flushing);` -- a bare
    // busy-spin for the whole 3.84 ms of every transfer, roughly 27 ms per frame
    // of burned CPU at priority 6, starving the buzzer at priority 5.
    // LVGL-UI-technical-challenges.md trap #17.
    lv_display_set_flush_wait_cb(lvDisplay_, &UiManager::flushWaitCb);

    display_->setTransferDoneCallback(&UiManager::onTransferDone, this);

    ESP_LOGI(TAG, "LVGL display %dx%d, 2 buffer x %u byte",
             static_cast<int>(geo.width), static_cast<int>(geo.height),
             static_cast<unsigned>(display_->flushBufferBytes()));
    return ESP_OK;
}

esp_err_t UiManager::initInput()
{
    if (touch_ == nullptr) {
        ESP_LOGW(TAG, "khong co cam ung -- giao dien chi hien thi");
        return ESP_OK;
    }

    lvIndev_ = lv_indev_create();
    if (lvIndev_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    lv_indev_set_type(lvIndev_, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lvIndev_, &UiManager::touchReadCb);
    lv_indev_set_user_data(lvIndev_, this);
    lv_indev_set_display(lvIndev_, lvDisplay_);

    // TAKE THE INPUT CLOCK AWAY FROM LVGL AND DRIVE IT FROM THE LOOP.
    //
    // By default lv_indev_create() gives the input its OWN lv_timer at
    // LV_DEF_REFR_PERIOD (33 ms), completely independent of the display refresh
    // timer -- lv_refr.c never reads an input device. Two 33 ms timers with no
    // fixed phase relationship means a fresh finger position can sit unread
    // until after the frame that would have shown it, costing an entire period
    // for no reason anyone can see in the code.
    //
    // Pausing it and calling lv_indev_read() ourselves, immediately before the
    // apps tick and the render, makes the order deterministic:
    //
    //     read input -> app reacts -> render
    //
    // all inside one pass. See taskBody().
    lv_timer_pause(lv_indev_get_read_timer(lvIndev_));
    return ESP_OK;
}

// ------------------------------------------------------------- flush plumbing

void UiManager::flushCb(lv_display_t* disp, const lv_area_t* area, uint8_t* px)
{
    auto* self = static_cast<UiManager*>(lv_display_get_user_data(disp));

    // lv_area_t is INCLUSIVE at both ends; display::Area has x1/y1 EXCLUSIVE.
    // Feeding one straight into the other silently drops the last row and
    // column of EVERY flush -- almost invisible on 240x280, which is exactly
    // what makes it dangerous.
    const display::Area a =
        display::fromInclusive(area->x1, area->y1, area->x2, area->y2);

    // RESERVE BEFORE DRAWING. The DMA can finish before the next line runs --
    // a small dirty rectangle is under a millisecond -- and if the flush has
    // not been recorded by then the ISR finds nothing pending, refuses a real
    // completion, and LVGL is never told. Measured on the board as
    // "bat dau=56 isr=55 thua=1 can bang=KHONG".
    if (!self->flush_.reserve()) {
        lv_display_flush_ready(disp);   // blocked after a timeout
        return;
    }

    // Every path out of this function must settle the flush exactly once.
    // drawRgb565() returns ESP_ERR_INVALID_STATE while the panel is asleep --
    // on purpose, so a lost frame is not silent -- but LVGL waits for
    // flush_ready unconditionally. Returning without reporting freezes the UI
    // at the first sleep, with the task still alive and nothing logged.
    if (self->display_->drawRgb565(a, px) != ESP_OK) {
        self->flush_.abandon();
        lv_display_flush_ready(disp);
    }
}

void UiManager::flushWaitCb(lv_display_t* disp)
{
    auto* self = static_cast<UiManager*>(lv_display_get_user_data(disp));
    if (!self->flush_.pending()) {
        return;
    }

    // Blocks on the semaphore the DMA ISR gives, rather than spinning. Same
    // reasoning as display.md section 7.3: spinning at priority 6 eats the CPU
    // of every lower-priority task.
    if (self->display_->waitIdle(self->cfg_.flushTimeoutMs) != ESP_OK) {
        // A 40-line transfer is 3.84 ms. Timing out means the panel or the DMA
        // channel is dead, not busy.
        //
        // Dangerous detail: once this returns, LVGL sets disp->flushing = 0 and
        // carries on -- it will render into that buffer again while the DMA may
        // still be reading it. So the coordinator blocks further flushes rather
        // than merely logging.
        self->flush_.onTimeout();
        ESP_LOGE(TAG, "flush qua han %" PRIu32 " ms -- chan flush moi",
                 self->cfg_.flushTimeoutMs);
    }
}

bool UiManager::onTransferDone(void* ctx)
{
    // ISR CONTEXT. No logging, no allocation, nothing without a FromISR suffix.
    auto* self = static_cast<UiManager*>(ctx);

    // Refuses a completion nobody is waiting for -- a stale callback from an
    // abandoned transfer, or a driver reporting twice. Reporting that one to
    // LVGL would release a buffer belonging to the NEXT flush.
    if (self->flush_.completeFromIsr()) {
        // Safe here: lv_display_flush_ready() is a single store (lv_display.c
        // sets disp->flushing = 0) and wakes nobody.
        lv_display_flush_ready(self->lvDisplay_);
    }

    // The return value IS the portYIELD_FROM_ISR flag. Nothing was woken here;
    // the UI task is released by the semaphore inside waitIdle().
    return false;
}

// --------------------------------------------------------------------- input

void UiManager::touchReadCb(lv_indev_t* indev, lv_indev_data_t* data)
{
    auto* self = static_cast<UiManager*>(lv_indev_get_user_data(indev));
    if (self->touch_ == nullptr) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    touch::TouchTransition t{};
    if (self->touch_->popTransition(t)) {
        self->lastPoint_.x = t.x;
        self->lastPoint_.y = t.y;
        self->pressed_     = (t.kind != touch::TransitionKind::Up);
        self->pointChanged_ = true;

        // How old was this sample when we got to it? The touch task stamped it
        // on arrival, so this is purely the queue-to-UI delay -- the half of
        // touch latency that has nothing to do with rendering.
        const uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        const uint32_t age = (now >= t.timestampMs) ? (now - t.timestampMs) : 0u;
        if (age > self->inputAgeMaxMs_) {
            self->inputAgeMaxMs_ = age;
        }
        self->inputAgeSumMs_ += age;
        ++self->inputAgeCount_;

        // Drain the queue rather than taking one transition per frame,
        // otherwise a drag lags behind the finger once the queue backs up.
        data->continue_reading = true;
    } else {
        data->continue_reading = false;
    }

    // The LAST VALID point, even when reporting a release: LVGL decides which
    // widget was clicked from the coordinate that arrives with RELEASED.
    // Reporting (0,0) there produces phantom clicks in the top-left corner.
    data->point = self->lastPoint_;
    data->state = self->pressed_ ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;

    // No I2C here. Every transaction belongs to the touch task at priority 11;
    // this only pops a queue, which is why the UI task stays out of the I2C
    // priority-inversion problem in i2c-bus-design.md section 11.1.
}

// -------------------------------------------------------------------- screen

// -------------------------------------------------------------------- runtime

esp_err_t UiManager::start()
{
    if (state_.load() != RuntimeState::Starting) {
        return ESP_ERR_INVALID_STATE;
    }
    const BaseType_t ok =
        xTaskCreate(&UiManager::taskTrampoline, "ui", cfg_.taskStackSize, this,
                    cfg_.taskPriority, &task_);
    if (ok != pdPASS) {
        state_.store(RuntimeState::Failed);
        return ESP_ERR_NO_MEM;
    }
    // From here on, this handle is the definition of "the UI task" for every
    // assertUiContext() check in the system.
    setUiTask(task_);

    // Let the touch task poke us the moment a transition is queued. Without
    // this the loop below sleeps up to maxIdleMs, and the FIRST touch after an
    // idle period waits for that sleep to expire -- measured at 764 ms against
    // a chip that reports every 12 ms.
    if (touch_ != nullptr) {
        touch_->setWakeTarget(task_);
    }

    state_.store(RuntimeState::Running);
    return ESP_OK;
}

void UiManager::taskTrampoline(void* arg) { static_cast<UiManager*>(arg)->taskBody(); }

void UiManager::taskBody()
{
    ESP_LOGI(TAG, "UI task chay, uu tien %u, stack %" PRIu32 " byte",
             static_cast<unsigned>(cfg_.taskPriority), cfg_.taskStackSize);

    for (;;) {
        // 1. Commands from other tasks, BEFORE LVGL draws, so a WAKE that
        //    arrived during this frame takes effect in this frame.
        drainCommands();

        // 2. The redraw the power policy asked for. GRAM contents are undefined
        //    across SLPIN/SLPOUT, so this must happen before the backlight comes
        //    back up or the user sees a frame of rubbish. display.md 11.2 rule 4.
        if (display_->takeRedrawRequest()) {
            lv_obj_invalidate(lv_screen_active());
        }

        // 3. Sample the finger. Deterministically here, not on LVGL's own
        //    schedule -- see initInput(). Everything below reacts to input that
        //    is at most one loop pass old.
        pointChanged_ = false;
        if (lvIndev_ != nullptr) {
            lv_indev_read(lvIndev_);
        }

        // 4. The foreground app gets its frame, with input already fresh. Only
        //    the foreground one: a background app runs nothing (app.hpp rule 4).
        host_.tick(static_cast<uint32_t>(esp_timer_get_time() / 1000));

        // 5. PAINT ON ARRIVAL, not on a schedule.
        //
        // The refresh timer was the largest remaining term in touch latency: a
        // position sampled a millisecond ago still waited for the timer to come
        // due. Repainting as soon as a new point exists removes that wait
        // entirely, and it cannot run away with the bus -- the chip only
        // produces a position every ~12 ms, and a moving marker dirties two
        // small rectangles, not the screen.
        //
        // Only when something actually changed. Without that guard this would
        // redraw identical pixels at the loop rate.
        if (cfg_.refreshOnTouchMove && pressed_ && pointChanged_) {
            lv_refr_now(lvDisplay_);
            ++forcedRefreshes_;
        }

        // 6. LVGL. flush_cb happens inside here.
        const uint32_t idleMs = lv_timer_handler();
        ++frames_;

        // Sampled here, in the UI task, so any task can read the numbers.
        lv_mem_monitor_t mem{};
        lv_mem_monitor(&mem);
        lvglFreeBytes_    = static_cast<uint32_t>(mem.free_size);
        lvglLargestBytes_ = static_cast<uint32_t>(mem.free_biggest_size);

        // 7. Sleep for what LVGL asked for, capped. Calling back sooner only
        //    burns CPU walking a timer list that is not due.
        uint32_t wait = idleMs;
        if (wait > cfg_.maxIdleMs) {
            wait = cfg_.maxIdleMs;
        }

        // WHILE A FINGER IS DOWN, SAMPLE FAR MORE OFTEN THAN WE DRAW.
        //
        // LVGL would have us sleep until the next refresh is due, up to 33 ms.
        // The render genuinely cannot go faster than that, but the finger
        // position it renders CAN be fresher: sampling every few milliseconds
        // means the point drawn is a few milliseconds old instead of a whole
        // frame old. That difference is the visible part of touch latency --
        // the eye reads a marker that trails the finger as lag long before it
        // reads 30 fps as choppy.
        //
        // Costs nothing when nobody is touching, which is almost always.
        if (pressed_ && wait > cfg_.touchPollMs) {
            wait = cfg_.touchPollMs;
        }

        // And paint faster too, for exactly as long as the finger is down.
        //
        // Sampling the finger every 6 ms achieves nothing if the result waits
        // up to 33 ms to be drawn -- after the ordering fix above, that wait
        // became the LARGEST remaining term in touch latency. Raising the
        // refresh rate only while touching buys responsiveness where it is felt
        // and leaves idle power untouched.
        if (pressed_ != fastRefresh_) {
            fastRefresh_ = pressed_;
            lv_timer_set_period(lv_display_get_refr_timer(lvDisplay_),
                                fastRefresh_ ? cfg_.touchRefreshMs
                                             : LV_DEF_REFR_PERIOD);
        }
        if (wait == 0) {
            wait = 1;
        }

        // Sleep on a NOTIFICATION, not a plain delay.
        //
        // vTaskDelay() sleeps out the full period no matter what arrives, so a
        // finger landing one millisecond in was not looked at until the sleep
        // ended. The notification is latched, so a poke that lands while this
        // task is busy rendering is not lost -- it simply makes the next take
        // return immediately.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait));
    }
}

void UiManager::drainCommands()
{
    UiCommand cmd{};

    // Critical first, and drained completely: an alarm must not wait behind a
    // burst of navigation.
    while (xQueueReceive(criticalQueue_, &cmd, 0) == pdTRUE) {
        applyCommand(cmd);
    }
    while (xQueueReceive(controlQueue_, &cmd, 0) == pdTRUE) {
        applyCommand(cmd);
    }
}

void UiManager::applyCommand(const UiCommand& cmd)
{
    switch (cmd.type) {
    case UiCommandType::WAKE:
        display_->exitSleep();
        break;
    case UiCommandType::SLEEP:
        display_->enterSleep();
        break;
    case UiCommandType::DIM:
        display_->setDimmed(true);
        break;
    case UiCommandType::SET_BRIGHTNESS:
        display_->setBrightness(cmd.arg, 150);
        break;
    case UiCommandType::INVALIDATE_SCREEN:
        lv_obj_invalidate(lv_screen_active());
        break;
    case UiCommandType::SHOW_APP:
        host_.showApp(cmd.arg);
        break;
    case UiCommandType::GO_BACK:
        host_.goBack();
        break;
    case UiCommandType::GO_HOME:
        host_.goHome();
        break;
    default:
        // SHOW_APP and the critical types need the app framework and the
        // services that raise them; logged rather than silently ignored so a
        // sender finds out its command went nowhere.
        ESP_LOGW(TAG, "lenh chua xu ly: %s", toString(cmd.type));
        break;
    }
}

// ---------------------------------------------------------------------- post

PostResult UiManager::post(const UiCommand& cmd)
{
    if (!acceptsCommands(state_.load())) {
        return PostResult::Rejected;
    }

    UiCommand copy = cmd;   // by value; never a pointer into the sender's frame
    copy.sequence  = sequence_.fetch_add(1) + 1;

    QueueHandle_t q =
        (laneOf(cmd.type) == Lane::Critical) ? criticalQueue_ : controlQueue_;

    // Timeout 0. A producer must NEVER block here: the IMU task may be holding
    // the I2C lock, and stalling it on a UI queue turns a slow interface into a
    // dead sensor.
    if (xQueueSend(q, &copy, 0) != pdTRUE) {
        ++dropped_;
        return PostResult::QueueFull;
    }
    return PostResult::Accepted;
}

PostResult UiManager::postFromIsr(const UiCommand& cmd, BaseType_t* higherWoken)
{
    if (!acceptsCommands(state_.load())) {
        return PostResult::Rejected;
    }

    UiCommand copy = cmd;
    copy.sequence  = sequence_.fetch_add(1) + 1;

    QueueHandle_t q =
        (laneOf(cmd.type) == Lane::Critical) ? criticalQueue_ : controlQueue_;

    if (xQueueSendFromISR(q, &copy, higherWoken) != pdTRUE) {
        ++dropped_;
        return PostResult::QueueFull;
    }
    return PostResult::Accepted;
}

// --------------------------------------------------------------- diagnostics

uint32_t UiManager::taskStackHeadroom() const
{
    if (task_ == nullptr) {
        return 0;
    }
    return uxTaskGetStackHighWaterMark(task_) * sizeof(StackType_t);
}

void UiManager::logDiagnostics(const char* where) const
{
    // flush/frame is the number that catches a screen redrawing itself for no
    // reason. With a static screen it belongs near 0; the first on-board run
    // sat at 2.00 and that is what exposed the unconditional lv_label_set_text.
    const uint32_t perFrame100 =
        (frames_ != 0) ? (flush_.started() * 100u / frames_) : 0u;
    ESP_LOGI(TAG,
             "[%s] %s  frame=%" PRIu32 "  flush/frame=%" PRIu32 ".%02" PRIu32
             "  stack con=%" PRIu32 " byte",
             where, toString(state_.load()), frames_, perFrame100 / 100u,
             perFrame100 % 100u, taskStackHeadroom());

    // The flush invariant, printed as one boolean. If this ever reads KHONG,
    // every pixel after it is suspect.
    ESP_LOGI(TAG,
             "  flush: bat dau=%" PRIu32 " isr=%" PRIu32 " tu-choi=%" PRIu32
             " bi-chan=%" PRIu32 " qua-han=%" PRIu32 " thua=%" PRIu32 "  can bang=%s",
             flush_.started(), flush_.completedByIsr(), flush_.readyNowRejected(),
             flush_.readyNowBlocked(), flush_.timedOut(), flush_.spurious(),
             flush_.balanced() ? "CO" : "KHONG");

    // Cached values, not a live lv_mem_monitor() call: this function runs in
    // the diagnostics loop, not the UI task.
    ESP_LOGI(TAG, "  LVGL heap: con %" PRIu32 " byte, manh lon nhat %" PRIu32,
             lvglFreeBytes_, lvglLargestBytes_);

    ESP_LOGI(TAG, "  SRAM noi: con %u byte, manh lon nhat %u, thap nhat tung co %u",
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)),
             static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)));

    if (inputAgeCount_ != 0) {
        ESP_LOGI(TAG,
                 "  cham: tuoi mau tb %" PRIu32 " ms, max %" PRIu32
                 " ms  (%" PRIu32 " mau, %" PRIu32 " lan ve tuc thi)",
                 inputAgeSumMs_ / inputAgeCount_, inputAgeMaxMs_, inputAgeCount_,
                 forcedRefreshes_);
    }

    if (dropped_ != 0) {
        ESP_LOGW(TAG, "  %" PRIu32 " lenh bi bo do queue day", dropped_);
    }

    host_.logDiagnostics(where);
}

}  // namespace ui
