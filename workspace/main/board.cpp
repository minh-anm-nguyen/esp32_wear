#include "board.hpp"

#include "display_selftest.hpp"

#include <cinttypes>

#include "driver/ledc.h"  // self-test only: reads the peripheral back
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace board {
namespace {

constexpr const char* TAG = "board";

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

// ---------------------------------------------------------------- initDevices

esp_err_t Board::initDevices(const Options& opts)
{
    // Sampled FIRST, before anything reconfigures the pad. GPIO0 is a
    // strapping pin and comes out of reset as an input with a pull-up, so
    // "held down at boot" reads as LOW without any setup. That gives an
    // entry gesture for calibration with no UI at all.
    calibrationRequested_ = (gpio_get_level(kButtonPin) == 0);

    // The GPIO ISR service is a system-wide singleton, and calling
    // gpio_install_isr_service() twice makes IDF log an ERROR even though the
    // second caller handles the return value fine. Install it ONCE here, then
    // tell both managers not to try -- which also stops the flags being an
    // accident of whichever component happens to start first.
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_install_isr_service(0));

    ESP_LOGI(TAG, "tick rate = %d Hz, 1 tick = %d ms",
             static_cast<int>(configTICK_RATE_HZ),
             static_cast<int>(portTICK_PERIOD_MS));

    const esp_err_t err = initButtonAndBuzzer(opts);
    if (err != ESP_OK) {
        return err;  // no sound and no input is the one state worth failing on
    }

    initDisplay(opts);
    initI2c();
    initImu();
    initTouch(opts);

    return ESP_OK;
}

esp_err_t Board::initButtonAndBuzzer(const Options& opts)
{
    // Wiring and behaviour are separate structs on purpose: the first describes
    // this board, the second describes how the button should feel. Only the
    // second one reaches the pure-logic layer.
    //
    // Filled in here but not APPLIED here -- addButton() runs in startSensors(),
    // after the self-tests, so a finger resting on BOOT during six seconds of
    // bring-up does not queue up a burst of gestures to be delivered at once.
    bwire_.pin                = kButtonPin;
    bwire_.activeLow          = true;   // button wired to GND
    bwire_.enableInternalPull = true;
    bwire_.enableWakeup       = false;  // no light sleep in this demo yet

    bhow_.enableDoubleClick     = true;   // on, so all the gesture types show up
    bhow_.enablePressDown       = true;   // feedback must not wait for doubleClickMs
    bhow_.enableDoubleClickHold = true;   // double click then hold -> DOUBLE_CLICK_HOLD
    bhow_.longPressMs           = 800;
    bhow_.doubleClickMs         = 250;
    bhow_.debounceMs            = 20;     // real time, correct at any tick rate

    zwire_.pin       = kBuzzerPin;
    zwire_.activeLow = false;

    zspec_.type      = buzzer::BuzzerType::PASSIVE;  // ACTIVE for a self-oscillating
                                                     // buzzer or a vibration motor
    zspec_.maxVolume = 80;   // a bare piezo driven flat out at 3.3V is painful up close
    zspec_.minFreqHz = 100;
    zspec_.maxFreqHz = 10000;

    // Sound comes up FIRST, before the I2C section. Axis calibration can sit
    // there for half a minute waiting to be told the watch is still, and a
    // wrist raise during that window should still be audible.
    esp_err_t err = buz_.init(zwire_, zspec_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "coi khong khoi tao duoc: %s", esp_err_to_name(err));
        return err;
    }
    err = buz_.start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "coi khong chay duoc: %s", esp_err_to_name(err));
        return err;
    }

    if (opts.runSelfTests) {
        buzzerSelfTest(buz_, zwire_, zspec_);
    }
    return ESP_OK;
}

void Board::initDisplay(const Options& opts)
{
    // The screen comes up next, and the ORDER RELATIVE TO THE BUZZER matters for
    // a reason that has nothing to do with either device: both configure a LEDC
    // timer, and on the ESP32-S3 the LEDC clock source is global to the whole
    // peripheral. They now agree on LEDC_USE_XTAL_CLK so either order works --
    // but if one of them ever drifts back to LEDC_AUTO_CLK, whichever runs
    // second dies with a bare ESP_FAIL from ledc_timer_config().
    // See doc-design/display.md section 10.2.
    const display::Wiring dwire{};  // the defaults already describe this board

    const esp_err_t derr = disp_.init(dwire, dcfg_);
    if (derr != ESP_OK) {
        ESP_LOGE(TAG, "khong khoi tao duoc man hinh: %s -- phan con lai van chay",
                 esp_err_to_name(derr));
        return;
    }
    displayReady_ = true;

    if (!opts.runSelfTests) {
        return;
    }

    // Self-proving bring-up, like buzzerSelfTest() above and logScan() below.
    // MISO is not wired on this board, so the firmware cannot read GRAM back:
    // the colour and orientation checks are for your eyes, and the log says what
    // each pattern should look like and which Config field to change when it
    // does not.
    display::SelfTestResult dst{};
    display::runSelfTest(disp_, dst, 800);

    if (!dst.callbackFired) {
        ESP_LOGE(TAG, "callback DMA khong chay -- flush cua LVGL se treo o buoc 5");
    }
    if (!dst.sleepWakeOk) {
        ESP_LOGE(TAG, "chu ky sleep/wake that bai sau %" PRIu32 " lan",
                 dst.sleepWakeCycles);
    }
}

void Board::initI2c()
{
    // The bus has exactly one owner, created here and handed to every driver
    // by reference. Nobody else calls i2c_new_master_bus().
    // See doc-design/i2c-bus-design.md section 2.
    i2c::BusManager::Config i2cCfg{};
    i2cCfg.sda = kI2cSda;
    i2cCfg.scl = kI2cScl;

    i2cReady_ = (i2cBus_.init(i2cCfg) == ESP_OK);
    if (!i2cReady_) {
        ESP_LOGE(TAG, "khong mo duoc bus I2C -- bo qua IMU, nut va coi van chay");
        return;
    }

    // Self-proving bring-up: the firmware asks the bus who is out there and
    // says so, naming what is MISSING as loudly as what is present. No meter,
    // no scope -- same idea as buzzerSelfTest() above.
    i2cBus_.logScan(TAG);
}

void Board::initImu()
{
    if (!i2cReady_) {
        return;
    }
    if (i2cBus_.createDevice(kImuAddr, 400000, imuDev_) != ESP_OK) {
        ESP_LOGE(TAG, "khong tao duoc thiet bi I2C 0x%02X", kImuAddr);
        return;
    }

    // ONE sink from the driver's point of view, many consumers from ours. The
    // fan-out is handed over now but is still EMPTY: services register with it
    // between initDevices() and startSensors().
    // doc-design/app-architecture.md section 3.
    imu::ImuManager::Config icfg{};
    icfg.bus               = &imuDev_;
    icfg.intPin            = kImuInt;
    icfg.sink              = &imuFanout_;
    icfg.installIsrService = false;  // Board owns it

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
    if (!calibrationRequested_) {
        icfg.remap.sign[2] = -1;
    }

    if (imu_.init(icfg) != ESP_OK) {
        ESP_LOGE(TAG, "IMU khong khoi tao duoc");
        return;
    }
    imuConfigured_ = true;
}

void Board::initTouch(const Options& opts)
{
    if (!i2cReady_) {
        return;
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
    if (i2cBus_.createDevice(kTouchAddr, 400000, touchDev_) != ESP_OK) {
        // Labelled, not just addressed: this message and the IMU one above are
        // otherwise identical and a reader should not have to remember which
        // address is which device.
        ESP_LOGE(TAG, "khong tao duoc thiet bi I2C 0x%02X (cam ung)", kTouchAddr);
        return;
    }

    touch::Wiring twire{};  // the defaults already describe V2.1

    touch::Behavior thow{};
    // BRING-UP ONLY. Prints every frame as raw hex plus its decode, which
    // is what settles the orientation flags and the event-flag semantics
    // from a captured log instead of from a guess. Turn this off once the
    // numbers are known: at 100 Hz during a drag it changes the timing it
    // is measuring.
    thow.logFrames = opts.logTouchFrames;

    touch::Geometry tgeo{};

    // A 180 DEGREE ROTATION. MEASURED on V2.1, 2026-08-31, from the launcher.
    //
    // The digitiser reports in the glass's own frame, and no document says how
    // that lines up with what the ST7789 puts on screen. This started as the
    // identity -- obviously wrong rather than subtly wrong -- and three
    // observations settled it:
    //
    //   tapping the top-left tile opened the bottom-centre app
    //   tapping the bottom-centre tile opened the top-left app
    //   tapping the top-right tile opened NOTHING; it needed the bottom-left
    //
    // The third one is what proves it. Under (x,y) -> (239-x, 279-y) the
    // top-right tile's centre maps to x=65, and the bottom tile starts at
    // x=66 -- it misses by a single pixel and lands in the gap. No other
    // combination of the three flags reproduces an unreachable tile, so this is
    // a measurement rather than a plausible guess.
    //
    // It also agrees with the panel: display::Geometry already sets
    // mirrorX/mirrorY true, so the glass and the LCD simply need the same
    // rotation. Obvious in hindsight, and only in hindsight.
    //
    // The LCD memory gap (display::Geometry::yGap = 20) is NOT part of this. It
    // describes where the visible window sits inside the ST7789's 240x320 of
    // GRAM, not where a finger is on the glass.
    tgeo.orientation.rawWidth  = dcfg_.geometry.width;
    tgeo.orientation.rawHeight = dcfg_.geometry.height;
    tgeo.orientation.mirrorX   = true;
    tgeo.orientation.mirrorY   = true;
    tgeo.orientation.swapXy    = false;
    tgeo.limits.width          = dcfg_.geometry.width;
    tgeo.limits.height         = dcfg_.geometry.height;

    if (touch_.init(touchDev_, twire, thow, tgeo) != ESP_OK
        || touch_.start() != ESP_OK) {
        ESP_LOGE(TAG, "cam ung khong khoi tao duoc");
        return;
    }
    touchReady_ = true;

    if (!opts.runSelfTests) {
        return;
    }

    // Self-proving bring-up, like buzzerSelfTest() and logScan() above.
    // The last argument asks the operator to touch the screen; frames
    // that arrive during that window are counted and printed.
    touch::SelfTestResult tst{};
    touch_.runSelfTest(tst, 4000);

    if (!tst.intIdleLevelOk) {
        ESP_LOGE(TAG, "INT GPIO14 khong nghi o muc mong doi -- moi ngat sau day "
                      "deu dang ngo");
    }
    if (!tst.touchObserved) {
        ESP_LOGW(TAG, "khong nhan duoc frame nao. Neu ban CO cham man hinh thi "
                      "xem lai IRQ (0xFA) va day noi INT, khong phai code doc frame");
    }
}

// --------------------------------------------------------------- startSensors

esp_err_t Board::startSensors()
{
    // Nothing may join the fan-out from here on: the IMU task is about to start
    // walking the array, and add() has no lock.
    imuFanout_.seal();
    ESP_LOGI(TAG, "fanout IMU: %u/%u bo tieu thu", imuFanout_.count(),
             imuFanout_.capacity());

    esp_err_t imuResult = ESP_OK;

    if (!imuConfigured_) {
        imuResult = ESP_ERR_INVALID_STATE;
    } else if (imu_.start() != ESP_OK) {
        ESP_LOGE(TAG, "IMU khong chay duoc");
        imuResult = ESP_FAIL;
    } else {
        imuReady_ = true;

        imu::SelfTestResult st{};
        imu_.runSelfTest(st);   // logs one PASS/FAIL line per check

        // Stay in ACTIVITY so samples keep flowing to the wrist FSM.
        // WOM would be the real watch idle state, but it produces no
        // sample stream to demonstrate with.
        imu_.setPowerMode(imu::PowerMode::ACTIVITY);

        if (calibrationRequested_) {
            // The remap cannot be guessed: nobody knows which way the chip was
            // glued until the board says so. Until this has been run once and
            // its output pasted into initImu(), the tilt angles are computed on
            // the wrong axes and a wrist raise may simply never register.
            imu::AxisRemap measured{};
            if (imu::runAxisCalibration(imu_, measured) == ESP_OK) {
                ESP_LOGI(TAG, "dan ket qua tren vao icfg.remap roi nap lai");
            }
        } else {
            ESP_LOGI(TAG, "giu nut BOOT luc khoi dong de hieu chuan truc");
        }
    }

    // THE BUTTON IS INDEPENDENT OF THE IMU and comes up either way.
    //
    // Not a stylistic point: the first version of this function returned early
    // when the IMU was missing, which left a board with a dead I2C bus also
    // holding a dead button -- contradicting the partial-failure rule stated in
    // board.hpp. Sequential bring-up steps must not inherit each other's
    // failures unless they genuinely depend on them.
    const esp_err_t berr = btn_.addButton(bwire_, bhow_);
    if (berr != ESP_OK) {
        ESP_LOGE(TAG, "khong them duoc nut: %s", esp_err_to_name(berr));
        return berr;
    }
    const esp_err_t serr = btn_.start();
    if (serr != ESP_OK) {
        ESP_LOGE(TAG, "nut khong chay duoc: %s", esp_err_to_name(serr));
        return serr;
    }

    return imuResult;
}

// ------------------------------------------------------------------ bus health

bool Board::logBusHealth(const char* why)
{
    struct Entry {
        const i2c::Device* dev;
        const char*        name;
    };
    const Entry entries[] = {
        {&touchDev_, "cam ung"},
        {&imuDev_, "IMU"},
    };

    bool printed = false;
    for (const auto& e : entries) {
        if (!e.dev->valid() || e.dev->totalErrors() == 0) {
            continue;  // healthy devices say nothing
        }
        if (!printed) {
            ESP_LOGW(TAG, "--- suc khoe I2C (%s) ---", why);
            printed = true;
        }
        // lastFailure() is kept separately from lastError() precisely so this
        // line still names the fault after the device has recovered.
        ESP_LOGW(TAG,
                 "  0x%02X %-8s: loi=%" PRIu32 "  chuoi dai nhat=%" PRIu32
                 "  lan cuoi: %s luc %" PRIu32 " ms%s",
                 e.dev->address(), e.name, e.dev->totalErrors(),
                 e.dev->worstErrorStreak(), esp_err_to_name(e.dev->lastFailure()),
                 e.dev->lastFailureMs(), e.dev->healthy() ? "" : "  [DANG HONG]");
    }
    return printed;
}

}  // namespace board
