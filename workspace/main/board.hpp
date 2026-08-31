// Every driver on the Waveshare ESP32-S3-Touch-LCD-1.69 V2.1, and the order
// they have to come up in.
//
// WHY THIS EXISTS
//
// app_main used to be three things at once: the composition root (what exists
// and how it is wired), the runtime (a loop draining queues), and product
// policy (which button makes which sound). The first is correct and should stay
// in one place. The other two are what made the file grow with every feature,
// and what would make it the file every future app has to edit.
//
// So: Board owns the hardware and the knowledge about bringing it up. Services
// owns the domain layer. main.cpp owns policy and is short enough to read in
// one screen. doc-design/app-architecture.md section 7.
//
// NOT A HARDWARE ABSTRACTION LAYER. There is one board. Board exists to give
// the init order a home, not to make the drivers swappable -- an abstraction
// over a single implementation is a cost with no buyer.
//
// TWO-PHASE ON PURPOSE
//
//   initDevices()   buses and drivers up, IMU configured but NOT running
//   <-- services register with imuSamples() here -->
//   startSensors()  seals the fan-out, starts the IMU task
//
// The gap in the middle is not an accident of layering: SampleFanout::add() is
// unsafe once the IMU task is walking the array, so every consumer has to be
// registered before the producer starts. Making it two calls puts that
// constraint in main.cpp where it can be seen, instead of in a comment nobody
// reads twice. See components/sensor_types/include/sample_fanout.hpp.
#pragma once

#include "button_manager.hpp"
#include "buzzer_manager.hpp"
#include "display_manager.hpp"
#include "i2c_device.hpp"
#include "imu_manager.hpp"
#include "sample_fanout.hpp"
#include "touch_manager.hpp"

#include "driver/gpio.h"
#include "esp_err.h"

namespace board {

// ------------------------------------------------------------------- wiring

// This board, V2.1. Values describe how the parts are soldered, not how they
// should behave -- the same split ButtonWiring/ButtonBehavior makes.
constexpr gpio_num_t kButtonPin = GPIO_NUM_0;   // BOOT; strapping pin, see below
constexpr gpio_num_t kBuzzerPin = GPIO_NUM_42;  // GPIO_NUM_33 on the older revision
constexpr gpio_num_t kI2cSda    = GPIO_NUM_11;  // shared: touch + RTC + IMU
constexpr gpio_num_t kI2cScl    = GPIO_NUM_10;
constexpr gpio_num_t kImuInt    = GPIO_NUM_38;  // QMI_INT2 (INT1 is not brought out)
constexpr uint16_t   kImuAddr   = 0x6B;
constexpr uint16_t   kTouchAddr = 0x15;         // CST816T, RST=GPIO13, INT=GPIO14

// How many things may consume the IMU stream at once.
//
// Four, for one consumer today, because the cost of a spare slot is one pointer
// and the cost of running out is editing this file under pressure. A watch
// grows these quickly: wrist raise (here now), step counting, sleep tracking,
// gesture recognition. doc-design/app-architecture.md section 3.
constexpr uint8_t kMaxImuConsumers = 4;

using ImuFanout = sensors::SampleFanout<kMaxImuConsumers>;

// ------------------------------------------------------------------ options

struct Options {
    // Self-proving bring-up: each driver is asked to demonstrate itself and the
    // log says what SHOULD have happened, so a failure names its own cause.
    // Costs about six seconds of boot, most of it the touch panel waiting for a
    // finger, so it is a flag rather than a fact.
    bool runSelfTests{true};

    // Prints every touch frame as raw hex plus its decode.
    //
    // OFF, because its job is done and it had started lying. It existed to
    // settle the orientation flags, which it did -- one drag showed the marker
    // landing at the point-mirror of the finger and that pinned mirrorX and
    // mirrorY in initTouch().
    //
    // Left on afterwards it distorted the very measurement it was feeding: at
    // 83 Hz the frame lines saturated USB-CDC and whole stretches of a drag
    // vanished from the log while the counters proved no frame was ever lost.
    // A diagnostic that drops its own output is worse than no diagnostic.
    //
    // Turn back on only to re-measure orientation, and only briefly.
    bool logTouchFrames{false};
};

// --------------------------------------------------------------------- board

class Board {
public:
    Board()                        = default;
    Board(const Board&)            = delete;
    Board& operator=(const Board&) = delete;

    // Brings up: GPIO ISR service, button, buzzer, display, I2C bus, IMU
    // (configured, not started), touch.
    //
    // Never returns a hard failure for a missing peripheral, and that is
    // deliberate -- see the note on partial failure below. The return value
    // covers only what makes the whole board useless.
    esp_err_t initDevices(const Options& opts = {});

    // Seals the fan-out and starts the IMU task. Call AFTER every service has
    // registered with imuSamples().
    esp_err_t startSensors();

    // Where services subscribe to the raw sample stream. Valid after
    // initDevices(), must not be touched after startSensors().
    ImuFanout& imuSamples() { return imuFanout_; }

    button::ButtonManager&   button() { return btn_; }
    buzzer::BuzzerManager&   buzzer() { return buz_; }
    display::DisplayManager& display() { return disp_; }
    touch::TouchManager&     touch() { return touch_; }
    imu::ImuManager&         imu() { return imu_; }
    i2c::BusManager&         i2c() { return i2cBus_; }

    // PARTIAL FAILURE IS THE NORMAL CASE, not an error path.
    //
    // A watch that cannot reach its IMU should still be a watch: it should still
    // buzz, still show a screen, and still say loudly what is missing. So none
    // of these use ESP_ERROR_CHECK -- that aborts, the chip reboots, and the one
    // useful error message scrolls past inside an endless boot loop while the
    // buzzer replays its self-test on every cycle.
    bool i2cReady() const { return i2cReady_; }
    bool displayReady() const { return displayReady_; }
    bool imuReady() const { return imuReady_; }
    bool touchReady() const { return touchReady_; }

    // True when BOOT was held at power-up. Sampled before any pad is
    // reconfigured, so it is only meaningful after initDevices().
    bool calibrationRequested() const { return calibrationRequested_; }

    const display::Config& displayConfig() const { return dcfg_; }

    // Per-device I2C error history -- printed ONLY when a device has actually
    // failed at least once. Returns true if it printed anything.
    //
    // Silent while healthy on purpose. A "0 loi" line every 10 s trains whoever
    // reads the log to skip the whole block, which is precisely the habit that
    // let a lone "I2C transaction timeout detected" go unattributed: the IDF
    // driver prints that without an address, and Device::consecutiveErrors_
    // had already reset to 0 by the next successful poll 16 ms later.
    bool logBusHealth(const char* why);

    // Read by main.cpp only to print the gesture timings it is telling the user
    // about, so the message and the behaviour cannot drift apart.
    const button::ButtonBehavior& buttonBehavior() const { return bhow_; }

private:
    esp_err_t initButtonAndBuzzer(const Options& opts);
    void      initDisplay(const Options& opts);
    void      initI2c();
    void      initImu();
    void      initTouch(const Options& opts);

    // ButtonManager takes its Config at construction and has no setter, so this
    // cannot wait until initDevices(). installIsrService must be false here:
    // Board installs the system-wide GPIO ISR service itself, and letting a
    // manager do it makes the flags an accident of which component starts first.
    static button::ButtonManager::Config buttonConfig()
    {
        button::ButtonManager::Config c{};
        c.installIsrService = false;
        return c;
    }

    button::ButtonManager   btn_{buttonConfig()};
    buzzer::BuzzerManager   buz_{};
    display::DisplayManager disp_{};
    i2c::BusManager         i2cBus_{};
    imu::ImuManager         imu_{};
    touch::TouchManager     touch_{};

    // Devices the application owns and the drivers borrow. They must outlive
    // every driver holding a reference, which is why they live here and not in
    // the init functions.
    i2c::Device imuDev_{};
    i2c::Device touchDev_{};

    ImuFanout imuFanout_{};

    display::Config      dcfg_{};
    buzzer::BuzzerWiring zwire_{};
    buzzer::BuzzerSpec   zspec_{};
    button::ButtonWiring bwire_{};
    button::ButtonBehavior bhow_{};

    bool calibrationRequested_{false};
    bool i2cReady_{false};
    bool displayReady_{false};
    bool imuReady_{false};
    bool touchReady_{false};
    bool imuConfigured_{false};
};

}  // namespace board
