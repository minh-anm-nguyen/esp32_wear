// Demo for the button + buzzer components.
//
// GPIO0 is the BOOT button on most ESP32-S3 devkits, so this is testable right
// after flashing. GPIO0 is a strapping pin: fine to use, just remember it selects
// the boot mode at reset time (see section 11 of the design document).
//
// The buzzer sits on GPIO42, which is where the V2.1 pinout of this board puts
// it. Boards from the older revision use GPIO33 instead, so check the silkscreen
// before blaming the firmware.
//
// GPIO42 is MTMS, one of the four JTAG pins (39 MTCK, 40 MTDO, 41 MTDI, 42 MTMS).
// That is harmless here because the ESP32-S3 debugs over its built-in USB
// Serial/JTAG on GPIO19/20; it only matters if you wire an external JTAG probe
// to the MTxx pins, which would then fight the buzzer for the pad.
//
// Driver stage: GPIO42 -> base resistor -> SS8050 NPN, buzzer between 3V3 and the
// collector. Active high, so activeLow stays false, and the pad must sit LOW
// whenever no note is playing or the transistor keeps conducting.
#include "button_manager.hpp"
#include "buzzer_manager.hpp"
#include "display_manager.hpp"
#include "display_selftest.hpp"
#include "i2c_device.hpp"
#include "imu_manager.hpp"
#include "motion_controller.hpp"
#include "touch_manager.hpp"

#include <cinttypes>

#include "driver/ledc.h"   // self-test only: reads the peripheral back
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "app";

constexpr gpio_num_t kButtonPin = GPIO_NUM_0;
constexpr gpio_num_t kI2cSda    = GPIO_NUM_11;  // dung chung: cam ung + RTC + IMU
constexpr gpio_num_t kI2cScl    = GPIO_NUM_10;
constexpr gpio_num_t kImuInt    = GPIO_NUM_38;  // QMI_INT2 (INT1 khong ra chan)
constexpr uint16_t   kImuAddr   = 0x6B;
constexpr uint16_t   kTouchAddr = 0x15;         // CST816T, RST=GPIO13, INT=GPIO14
constexpr gpio_num_t kBuzzerPin = GPIO_NUM_42;  // GPIO_NUM_33 on the older revision

const char* eventName(button::ButtonEvent e)
{
    switch (e) {
    case button::ButtonEvent::PRESS_DOWN:         return "PRESS_DOWN";
    case button::ButtonEvent::CLICK:              return "CLICK";
    case button::ButtonEvent::DOUBLE_CLICK:       return "DOUBLE_CLICK";
    case button::ButtonEvent::DOUBLE_CLICK_HOLD:  return "DOUBLE_CLICK_HOLD";
    case button::ButtonEvent::LONG_PRESS:         return "LONG_PRESS";
    case button::ButtonEvent::LONG_PRESS_RELEASE: return "LONG_PRESS_RELEASE";
    case button::ButtonEvent::NONE:               return "NONE";
    }
    return "?";
}

// One gesture, one sound. This table is the entire coupling between the two
// components: neither one references the other, the application joins them.
// Returning a pointer into flash is safe -- the stock patterns are constexpr,
// so they outlive any playback (see the lifetime note on buzzer::Pattern).
// Gesture -> sound, decided by the APPLICATION. components/motion has never
// heard of a buzzer and components/imu has never heard of MotionController;
// this callback is the only place all three meet. Same arrangement that
// already joins the button to the buzzer below.
void onMotionEvent(void* ctx, motion::MotionEvent ev, uint32_t nowMs)
{
    auto* buz = static_cast<buzzer::BuzzerManager*>(ctx);
    if (ev == motion::MotionEvent::WRIST_RAISE) {
        ESP_LOGI(TAG, "NANG CO TAY (t=%" PRIu32 " ms) -> bat man hinh", nowMs);
        buz->play(buzzer::patterns::kClick);
    } else if (ev == motion::MotionEvent::WRIST_LOWER) {
        ESP_LOGI(TAG, "ha tay (t=%" PRIu32 " ms) -> tat man hinh", nowMs);
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

// One second of continuous tone, played once at boot.
constexpr buzzer::Note kSelfTestNotes[] = {{2000, 1000, 100}};

// Separates the two failure modes that sound identical from the outside:
// firmware not driving the pin at all, versus a pin that carries the right
// square wave into hardware that cannot make a sound out of it.
//
// ledc_get_freq() and ledc_get_duty() read the peripheral registers back, so
// what they print is what the pin is actually doing, not what we asked for.
void buzzerSelfTest(buzzer::BuzzerManager&      buz,
                    const buzzer::BuzzerWiring& wiring,
                    const buzzer::BuzzerSpec&   spec)
{
    ESP_LOGI(TAG, "--- self test: 1 giay am 2000 Hz tren GPIO%d ---",
             static_cast<int>(wiring.pin));

    const esp_err_t err = buz.play(buzzer::makePattern(kSelfTestNotes, 1, 255));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "play() that bai: %s", esp_err_to_name(err));
        return;
    }

    // Let the buzzer task pick the command up and program LEDC before reading.
    vTaskDelay(pdMS_TO_TICKS(200));

    if (spec.type == buzzer::BuzzerType::ACTIVE) {
        ESP_LOGI(TAG, "  loai ACTIVE: muc GPIO = %d (mong doi %d)",
                 gpio_get_level(wiring.pin), wiring.activeLow ? 0 : 1);
    } else {
        const uint32_t freq = ledc_get_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
        const uint32_t duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ESP_LOGI(TAG, "  LEDC doc nguoc: freq=%" PRIu32 " Hz, duty=%" PRIu32 "/1024",
                 freq, duty);

        // volume 100 is clamped to maxVolume, and volume maps onto the lower half
        // of the duty range because a piezo is silent at 100% duty.
        const uint32_t wantDuty = 512u * spec.maxVolume / 100u;
        if (freq == 0 || duty == 0) {
            ESP_LOGE(TAG, "  LEDC KHONG chay -> loi phan mem, khong phai day noi");
        } else if (duty != wantDuty) {
            ESP_LOGW(TAG, "  duty la %" PRIu32 ", mong doi %" PRIu32, duty, wantDuty);
        } else {
            ESP_LOGI(TAG, "  LEDC dung: chan GPIO%d DANG co xung vuong. Neu khong "
                          "nghe thay gi thi van de nam o day noi hoac loa.",
                     static_cast<int>(wiring.pin));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // let the tone finish before the demo starts
}

}  // namespace

extern "C" void app_main(void)
{
    // Sampled FIRST, before anything reconfigures the pad. GPIO0 is a
    // strapping pin and comes out of reset as an input with a pull-up, so
    // "held down at boot" reads as LOW without any setup. That gives an
    // entry gesture for calibration with no UI at all.
    const bool calibrationRequested = (gpio_get_level(kButtonPin) == 0);

    // The GPIO ISR service is a system-wide singleton, and calling
    // gpio_install_isr_service() twice makes IDF log an ERROR even though the
    // second caller handles the return value fine. Install it ONCE here, then
    // tell both managers not to try -- which also stops the flags being an
    // accident of whichever component happens to start first.
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_install_isr_service(0));

    // pollIntervalMs derives itself from the tick rate: 10ms at HZ=100, 5ms at HZ=1000.
    static button::ButtonManager btn{[] {
        button::ButtonManager::Config c{};
        c.installIsrService = false;  // app_main owns it
        return c;
    }()};
    static buzzer::BuzzerManager buz;

    ESP_LOGI(TAG, "tick rate = %d Hz, 1 tick = %d ms",
             static_cast<int>(configTICK_RATE_HZ),
             static_cast<int>(portTICK_PERIOD_MS));

    // Wiring and behaviour are separate structs on purpose: the first describes
    // this board, the second describes how the button should feel. Only the
    // second one reaches the pure-logic layer.
    button::ButtonWiring bwire{};
    bwire.pin                = kButtonPin;
    bwire.activeLow          = true;   // button wired to GND
    bwire.enableInternalPull = true;
    bwire.enableWakeup       = false;  // no light sleep in this demo yet

    button::ButtonBehavior bhow{};
    bhow.enableDoubleClick = true;   // on, so all the gesture types show up
    bhow.enablePressDown   = true;   // feedback must not wait for doubleClickMs
    bhow.enableDoubleClickHold = true;  // double click then keep holding -> DOUBLE_CLICK_HOLD
    bhow.longPressMs       = 800;
    bhow.doubleClickMs     = 250;
    bhow.debounceMs        = 20;     // real time, correct at any tick rate

    buzzer::BuzzerWiring zwire{};
    zwire.pin       = kBuzzerPin;
    zwire.activeLow = false;

    buzzer::BuzzerSpec zspec{};
    zspec.type      = buzzer::BuzzerType::PASSIVE;  // ACTIVE for a self-oscillating
                                                    // buzzer or a vibration motor
    zspec.maxVolume = 80;   // a bare piezo driven flat out at 3.3V is painful up close
    zspec.minFreqHz = 100;
    zspec.maxFreqHz = 10000;

    // Sound comes up FIRST, before the I2C section. Axis calibration can sit
    // there for half a minute waiting to be told the watch is still, and a
    // wrist raise during that window should still be audible.
    ESP_ERROR_CHECK(buz.init(zwire, zspec));
    ESP_ERROR_CHECK(buz.start());
    buzzerSelfTest(buz, zwire, zspec);

    // The screen comes up next, and the ORDER RELATIVE TO THE BUZZER matters for
    // a reason that has nothing to do with either device: both configure a LEDC
    // timer, and on the ESP32-S3 the LEDC clock source is global to the whole
    // peripheral. They now agree on LEDC_USE_XTAL_CLK so either order works --
    // but if one of them ever drifts back to LEDC_AUTO_CLK, whichever runs
    // second dies with a bare ESP_FAIL from ledc_timer_config().
    // See doc-design/display.md section 10.2.
    //
    // Not ESP_ERROR_CHECK, same reasoning as the I2C bus below: a watch that
    // cannot bring up its screen should still buzz and still count steps, and a
    // boot loop would scroll the one useful error message past too fast to read.
    static display::DisplayManager disp;
    const display::Wiring dwire{};  // the defaults already describe this board
    display::Config       dcfg{};

    const esp_err_t derr = disp.init(dwire, dcfg);
    if (derr == ESP_OK) {
        // Self-proving bring-up, like buzzerSelfTest() above and logScan()
        // below. MISO is not wired on this board, so the firmware cannot read
        // GRAM back: the colour and orientation checks are for your eyes, and
        // the log says what each pattern should look like and which Config
        // field to change when it does not.
        display::SelfTestResult dst{};
        display::runSelfTest(disp, dst, 800);

        if (!dst.callbackFired) {
            ESP_LOGE(TAG, "callback DMA khong chay -- flush cua LVGL se treo o buoc 5");
        }
        if (!dst.sleepWakeOk) {
            ESP_LOGE(TAG, "chu ky sleep/wake that bai sau %" PRIu32 " lan",
                     dst.sleepWakeCycles);
        }
    } else {
        ESP_LOGE(TAG, "khong khoi tao duoc man hinh: %s -- phan con lai van chay",
                 esp_err_to_name(derr));
    }

    // The bus has exactly one owner, created here and handed to every driver
    // by reference. Nobody else calls i2c_new_master_bus().
    // See doc-design/i2c-bus-design.md section 2.
    static i2c::BusManager  i2cBus;
    i2c::BusManager::Config i2cCfg{};
    i2cCfg.sda = kI2cSda;
    i2cCfg.scl = kI2cScl;
    // NOT ESP_ERROR_CHECK. That aborts, the chip reboots, and the whole thing
    // becomes an endless boot loop in which the one error message scrolls past
    // too fast to read -- while the buzzer replays its self-test tone on every
    // cycle. A watch that cannot reach its IMU should still be a watch.
    const bool i2cReady = (i2cBus.init(i2cCfg) == ESP_OK);
    if (!i2cReady) {
        ESP_LOGE(TAG, "khong mo duoc bus I2C -- bo qua IMU, nut va coi van chay");
    }

    // Self-proving bring-up: the firmware asks the bus who is out there and
    // says so, naming what is MISSING as loudly as what is present. No meter,
    // no scope -- same idea as buzzerSelfTest() above.
    if (i2cReady) {
        i2cBus.logScan(TAG);
    }

    // The IMU borrows a Device the application owns. Static storage: the
    // Device must outlive every driver holding a reference to it.
    static i2c::Device imuDev;
    static imu::ImuManager imuMgr;
    if (i2cReady && i2cBus.createDevice(kImuAddr, 400000, imuDev) == ESP_OK) {
        // The IMU pushes samples at whatever implements sensors::ISampleSink.
        // It never learns that this one recognises wrist raises.
        static motion::MotionController wrist;
        wrist.setEventCallback(onMotionEvent, &buz);

        imu::ImuManager::Config icfg{};
        icfg.bus    = &imuDev;
        icfg.intPin = kImuInt;
        icfg.sink   = &wrist;
        icfg.installIsrService = false;  // app_main owns it
        // The chip is glued with its Z axis pointing INTO the screen, so the
        // sign has to be flipped. Measured on the bench, not guessed: with the
        // identity remap, laying the board glass-up reported a LOWER and
        // PCB-up reported a RAISE -- exactly backwards.
        //
        // The convention this restores (motion_controller.hpp): +Z points OUT
        // of the screen. Worn on a wrist with the glass facing outward, an arm
        // hanging by the side puts the screen normal roughly horizontal (~90
        // degrees, above downThresholdDeg = 65 -> screen off), and raising the
        // arm to read the time brings it to ~20-40 degrees (below
        // viewThresholdDeg = 35 -> screen on).
        //
        // X and Y are still identity. They do not affect wrist raise, which
        // only uses the Z component and the magnitude -- but screen rotation
        // will need them, so run the full calibration (hold BOOT at power-up)
        // before building that.
        //
        // Left at identity while calibrating: runAxisCalibration() can only
        // derive a mapping from SENSOR-frame readings, and latest() hands back
        // samples that have already been through the remap.
        if (!calibrationRequested) {
            icfg.remap.sign[2] = -1;
        }

        if (imuMgr.init(icfg) == ESP_OK && imuMgr.start() == ESP_OK) {
            imu::SelfTestResult st{};
            imuMgr.runSelfTest(st);   // logs one PASS/FAIL line per check

            // Stay in ACTIVITY so samples keep flowing to the wrist FSM.
            // WOM would be the real watch idle state, but it produces no
            // sample stream to demonstrate with.
            imuMgr.setPowerMode(imu::PowerMode::ACTIVITY);

            if (calibrationRequested) {
                // The remap cannot be guessed: nobody knows which way the
                // chip was glued until the board says so. Until this has
                // been run once and its output pasted below, the tilt
                // angles are computed on the wrong axes and a wrist raise
                // may simply never register.
                imu::AxisRemap measured{};
                if (imu::runAxisCalibration(imuMgr, measured) == ESP_OK) {
                    ESP_LOGI(TAG, "dan ket qua tren vao icfg.remap roi nap lai");
                }
            } else {
                ESP_LOGI(TAG, "giu nut BOOT luc khoi dong de hieu chuan truc");
            }

            ESP_LOGI(TAG, "nang co tay len ngang mat -> se keu mot tieng bip");
        } else {
            ESP_LOGE(TAG, "IMU khong khoi tao duoc");
        }
    } else {
        if (i2cReady) {
            ESP_LOGE(TAG, "khong tao duoc thiet bi I2C 0x%02X", kImuAddr);
        }
    }

    // The touch panel shares the bus with the IMU and the RTC, so it borrows a
    // Device the application owns, exactly like the IMU above.
    //
    // 400 kHz, not the 100 kHz that gets copied out of older Waveshare
    // examples. Section 4.5 of the CST816S datasheet gives the chip a
    // 10 kHz..400 kHz range and section 6.b recommends 400 kHz as the maximum
    // for reliable communication, and i2c-bus-design.md section 3 already
    // budgets this device at 400 kHz. A device speed is per-device on this
    // driver, so nothing else on the bus is affected either way.
    static i2c::Device        touchDev;
    static touch::TouchManager touchMgr;
    if (i2cReady && i2cBus.createDevice(kTouchAddr, 400000, touchDev) == ESP_OK) {
        touch::Wiring twire{};  // the defaults already describe V2.1

        touch::Behavior thow{};
        // BRING-UP ONLY. Prints every frame as raw hex plus its decode, which
        // is what settles the orientation flags and the event-flag semantics
        // from a captured log instead of from a guess. Turn this off once the
        // numbers are known: at 100 Hz during a drag it changes the timing it
        // is measuring.
        thow.logFrames = true;

        touch::Geometry tgeo{};
        // Deliberately the IDENTITY, and this is the one thing here that is
        // expected to be wrong.
        //
        // The digitiser reports in the glass's own frame, and nothing in any
        // document says how that lines up with what the ST7789 puts on screen
        // (dwire/dcfg currently ask for mirrorX and mirrorY). Guessing would
        // produce a panel that responds in a mirror image -- subtly wrong, and
        // hard to attribute to any one of the three flags. So it starts as the
        // identity, which is OBVIOUSLY wrong, and thow.logFrames prints
        // raw(x,y) -> (x,y) for every touch: one press in a known corner names
        // the right combination, and it becomes three booleans here.
        //
        // The LCD memory gap (display::Geometry::yGap = 20) is NOT part of
        // this. It describes where the visible window sits inside the ST7789's
        // 240x320 of GRAM, not where a finger is on the glass.
        tgeo.orientation.rawWidth  = dcfg.geometry.width;
        tgeo.orientation.rawHeight = dcfg.geometry.height;
        tgeo.limits.width          = dcfg.geometry.width;
        tgeo.limits.height         = dcfg.geometry.height;

        if (touchMgr.init(touchDev, twire, thow, tgeo) == ESP_OK
            && touchMgr.start() == ESP_OK) {
            // Self-proving bring-up, like buzzerSelfTest() and logScan() above.
            // The last argument asks the operator to touch the screen; frames
            // that arrive during that window are counted and printed.
            touch::SelfTestResult tst{};
            touchMgr.runSelfTest(tst, 4000);

            if (!tst.intIdleLevelOk) {
                ESP_LOGE(TAG, "INT GPIO14 khong nghi o muc mong doi -- moi ngat sau day "
                              "deu dang ngo");
            }
            if (!tst.touchObserved) {
                ESP_LOGW(TAG, "khong nhan duoc frame nao. Neu ban CO cham man hinh thi "
                              "xem lai IRQ (0xFA) va day noi INT, khong phai code doc frame");
            }
        } else {
            ESP_LOGE(TAG, "cam ung khong khoi tao duoc");
        }
    } else if (i2cReady) {
        ESP_LOGE(TAG, "khong tao duoc thiet bi I2C 0x%02X (cam ung)", kTouchAddr);
    }

    // Buttons last: a button already held down at start() is caught by the
    // very first poll, and everything that reacts to it -- the buzzer, the
    // calibration gesture -- must already exist by then.
    ESP_ERROR_CHECK(btn.addButton(bwire, bhow));
    ESP_ERROR_CHECK(btn.start());

    ESP_LOGI(TAG, "san sang. Nut GPIO%d, coi GPIO%d",
             static_cast<int>(kButtonPin), static_cast<int>(kBuzzerPin));
    ESP_LOGI(TAG, "  cham xuong   -> bip ngan NGAY (~20ms, chi qua debounce)");
    ESP_LOGI(TAG, "  double click -> hai tieng len giong");
    ESP_LOGI(TAG, "  giu %" PRIu32 "ms   -> bao thuc lap vo han (click luc nay bi bo qua)",
             bhow.longPressMs);
    ESP_LOGI(TAG, "  nha tay      -> tat bao thuc");
    ESP_LOGI(TAG, "luu y: CLICK van bi tre %" PRIu32 " ms vi FSM phai loai tru double "
                  "click; no khong keu, tieng bip da phat luc cham xuong roi",
             bhow.doubleClickMs);

    // Drain BOTH queues. Waiting forever on the button one left the IMU queue
    // to fill up and start dropping events -- which is exactly what the
    // "queue day, bo su kien 0" warnings were saying.
    button::ButtonEventMsg msg;
    uint32_t               lastTouchDiagMs = 0;
    for (;;) {
        imu::ImuEventMsg iev;
        while (imuMgr.waitEvent(iev, 0)) {
            ESP_LOGI(TAG, "IMU  %-15s  t=%" PRIu32 " ms", imuEventName(iev.type),
                     iev.timestampMs);
        }

        // Drained the way TouchLvglAdapter will drain it once LVGL is wired up:
        // pop until empty, every pass. Move collapses inside the tracker, so a
        // drag produces one entry per pass here rather than one per interrupt.
        touch::TouchTransition tt;
        while (touchMgr.popTransition(tt)) {
            ESP_LOGI(TAG, "CHAM %-4s  (%3d,%3d)  seq=%" PRIu32 "  t=%" PRIu32 " ms%s",
                     touch::toString(tt.kind), static_cast<int>(tt.x),
                     static_cast<int>(tt.y), tt.sequence, tt.timestampMs,
                     tt.synthetic ? "   [tu sinh]" : "");
        }

        // Counters, on a slow timer. The per-frame detail is in the raw log
        // above; this is the summary that answers "is the release timeout
        // right" and "is anything being dropped".
        if (touchMgr.isRunning()) {
            const uint32_t nowTick = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (nowTick - lastTouchDiagMs >= 10000) {
                lastTouchDiagMs = nowTick;
                touchMgr.logDiagnostics("dinh ky 10 s");
            }
        }

        if (!btn.waitEvent(msg, 100)) {
            continue;  // just a timeout: go back and check the IMU queue
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
            buz.silence();
            continue;
        }

        if (const buzzer::Pattern* sound = soundFor(msg.event)) {
            // Returns ESP_ERR_TIMEOUT if the command queue is full, which only
            // happens when someone presses far faster than a pattern can play.
            // Dropping a beep is the right answer there: this loop must not block.
            const esp_err_t err = buz.play(*sound);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "play() bo qua: %s", esp_err_to_name(err));
            }
        }
    }

    // unreachable: the loop above never exits
}
