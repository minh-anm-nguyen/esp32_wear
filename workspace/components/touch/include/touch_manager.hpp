// The ESP-IDF half of the touch panel: one task, one ISR, the reset line, the
// recovery policy and the queue the UI drains. Everything that knows about
// FreeRTOS lives here so that cst816t.hpp, touch_tracker.hpp and
// touch_transform.hpp stay compilable with a bare g++.
//
// OWNERSHIP, which is the whole reason this class exists:
//
//   - After start(), ONLY the touch task talks to the CST816T. PowerManager
//     and app_main send a mode request and wait for an acknowledgement; they
//     never reach for the driver. i2c::Device enforces this at runtime and
//     says so out loud when it is violated (i2c-bus-design.md section 7.1).
//   - The UI task only pops transitions and reads snapshots. It performs no
//     I2C, ever.
//   - A touch failure resets the CST816T through GPIO13 and NOTHING ELSE. The
//     shared bus carries the IMU and the RTC; recovering it because one device
//     went quiet would take two working chips down with the broken one. If the
//     bus itself is wedged, this class reports it and the application decides.
//
// WHY THE CHIP LOOKS DEAD WHEN NOBODY IS TOUCHING IT
//
// Datasheet 4.1: two seconds after the last touch the chip drops into standby.
// It still scans and still pulses the interrupt line, but it should not be
// expected to answer I2C. That is normal and must never be counted as a fault
// -- so health here is judged only on reads that follow an interrupt, never on
// whether the chip answers a probe while idle.
#pragma once

#include "cst816t.hpp"
#include "touch_tracker.hpp"
#include "touch_transform.hpp"

#include "i2c_device.hpp"

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <atomic>

namespace touch {

// ------------------------------------------------------------------- state

enum class State : uint8_t {
    Stopped,     // no task, no ISR; runtime resources must not be touched
    Starting,    // task up, chip being reset and configured
    Idle,        // running, no finger
    Tracking,    // a contact is in progress
    ChipSleeping,// chip parked in 0xE5 sleep; only a reset brings it back
    Recovering,  // interrupt masked, resetting and reapplying configuration
    Fault,       // fast retries stopped; diagnostics and explicit retry only
    Stopping,
};

const char* toString(State s);

// What the application can ask the touch task to be doing.
enum class Mode : uint8_t {
    Active,     // normal pointer input
    ChipSleep,  // 0xE5 = 0x03. Leaves only by reset -- see Cst816t::enterDeepSleep
};

// ------------------------------------------------------------------ wiring

// This board. Values here describe how the CST816T is soldered, not how it
// should behave -- that split is the same one ButtonWiring/ButtonBehavior make,
// and it exists because wiring is fixed for V2.1 while behaviour is a product
// decision that will change.
struct Wiring {
    gpio_num_t resetPin{GPIO_NUM_13};
    gpio_num_t intPin{GPIO_NUM_14};

    // Datasheet 4.3: "When the external reset pin RSTn is low, the whole chip
    // will be reset." Active low, with a built-in pull-up and an RC filter --
    // which is exactly why ResetTiming::assertMs is generous.
    bool resetActiveLow{true};

    // The register document describes every interrupt source as generating a
    // LOW PULSE, so the line idles high and falls to signal.
    bool intActiveLow{true};

    // UNVERIFIED whether the board fits an external pull-up on INT. An internal
    // one is harmless if there is: two pull-ups in parallel are still a
    // pull-up. Without either, a floating input produces phantom interrupts,
    // which is the worse failure.
    bool intInternalPullup{true};

    // app_main owns the GPIO ISR service (it calls gpio_install_isr_service(0)
    // once), and both other managers are told the same thing. Leaving this true
    // makes the service flags an accident of which component starts first.
    bool installIsrService{false};
};

// ---------------------------------------------------------------- behaviour

struct Behavior {
    ChipConfig    chip{};
    TrackerConfig tracker{};
    ResetTiming   reset{};

    uint32_t    taskStackSize{4096};

    // 11, the top of the I2C band. See i2c-bus-design.md section 11: touch 11,
    // IMU 10, buttons 9, UI 6. High priority here buys interrupt-to-read
    // latency and NOTHING else -- this task must be blocked essentially all the
    // time. A priority 11 task that polls would starve the IMU and the UI
    // rather than making touch feel faster.
    UBaseType_t taskPriority{11};
    BaseType_t  taskCoreId{tskNO_AFFINITY};

    // Wake-up interval when no interrupt arrives. This is what drives the
    // synthetic release, so it must be well below TrackerConfig::releaseTimeoutMs
    // or a lifted finger stays pressed until the next tick rather than until
    // the timeout.
    uint32_t idleTickMs{40};

    // Ceiling on the work done for one wake-up.
    //
    // BOTH a count and a duration, because they fail differently. The count
    // stops an interrupt storm from monopolising the task; the duration stops
    // a wedged bus from doing it, and that is the one that matters most:
    // i2c::kXferTimeoutMs is 50 ms and applies to EVERY transfer, so two failed
    // reads in a row hold the shared bus lock for 100 ms at priority 11 -- far
    // past the 5 ms per-transaction ceiling that i2c-bus-design.md section 8
    // sets to keep the IMU running.
    uint8_t  maxReadsPerWake{4};
    uint32_t readBudgetMs{60};

    // Consecutive interrupt-associated read failures before the chip is reset.
    // Only reads that FOLLOW an interrupt count: the chip not answering while
    // idle is standby, not a fault.
    uint8_t failuresBeforeRecovery{3};

    // Resets attempted before giving up and sitting in Fault. Without a budget
    // an interrupt line stuck active resets the chip forever: reset, immediate
    // interrupt, read fails, reset again.
    uint8_t  maxRecoveryAttempts{3};
    uint32_t recoveryBackoffMs{5000};

    // Bring-up only. Logs every frame as raw hex plus its decode, which is
    // exactly what settles the orientation flags and the event-flag semantics
    // from a captured log rather than from a guess.
    //
    // OFF by default: at 100 Hz during a drag this changes the timing it is
    // supposed to be measuring.
    bool     logFrames{false};
    uint32_t logFramesPerSecond{20};
};

// ----------------------------------------------------------------- geometry

struct Geometry {
    RawLimits        limits{};
    TouchOrientation orientation{};
};

// -------------------------------------------------------------- diagnostics

// Counters, not log lines. Logging each interrupt or each Move would change
// the timing being measured and drown the console; these are sampled on demand.
struct Diagnostics {
    State state{State::Stopped};

    uint32_t irqCount{0};
    uint32_t irqCoalesced{0};   // interrupts folded into one wake-up
    uint32_t reads{0};
    uint32_t readFailures{0};   // NACK or timeout on a read that followed an IRQ
    uint32_t validFrames{0};
    uint32_t invalidFrames{0};
    uint32_t allOnesFrames{0};
    uint32_t badFingerCount{0};
    uint32_t reservedEvent{0};
    uint32_t outOfRange{0};
    uint32_t edgeClamped{0};
    uint32_t transformRejected{0};
    uint32_t unknownGesture{0};
    uint32_t fingerEventMismatch{0};

    // From the tracker.
    uint32_t downs{0};
    uint32_t ups{0};
    uint32_t syntheticUps{0};
    uint32_t moves{0};
    uint32_t coalescedMoves{0};
    uint32_t queueOverflows{0};
    uint32_t missedUps{0};
    uint32_t eventUpWithFinger{0};

    uint32_t resets{0};
    uint32_t recoveries{0};
    uint32_t recoveryFailures{0};
    uint32_t intStuckActive{0};

    // Interrupt to first byte read, in microseconds. What justifies priority 11
    // -- or shows that it is not buying anything.
    uint32_t irqToReadMaxUs{0};
    uint32_t irqToReadAvgUs{0};

    // Gap between consecutive interrupts while a finger is down, in
    // milliseconds. THIS is what turns TrackerConfig::releaseTimeoutMs from a
    // guess into a measurement: the timeout wants to sit comfortably above
    // contactIrqMaxMs.
    uint32_t contactIrqMinMs{0};
    uint32_t contactIrqMaxMs{0};
    uint32_t contactIrqAvgMs{0};

    ChipInfo     chip{};
    ConfigResult lastConfig{};
};

// What runSelfTest() found. One PASS/FAIL line per field is logged as it goes,
// in the manner of display::runSelfTest() and ImuManager::runSelfTest().
struct SelfTestResult {
    bool resetDone{false};
    bool identityRead{false};
    bool identityPlausible{false};
    bool configWritten{false};
    bool configVerified{false};
    uint8_t firstBadConfigReg{0};

    bool intIdleLevelOk{false};   // line sits INACTIVE with no finger present
    int  intIdleLevel{-1};

    // Only attempted when Behavior::chip.irqSelfTest is set: the chip pulses
    // the interrupt on its own, proving the wiring and the ISR path with
    // nobody touching the glass.
    bool     irqSelfPulseTried{false};
    uint32_t irqSelfPulseCount{0};

    // Optional human step: touch the screen when asked.
    bool     touchObserved{false};
    uint32_t framesObserved{0};
};

// ------------------------------------------------------------- TouchManager

class TouchManager {
public:
    TouchManager() = default;
    ~TouchManager();

    TouchManager(const TouchManager&)            = delete;
    TouchManager& operator=(const TouchManager&) = delete;

    // Takes an i2c::Device rather than an i2c::Bus, unlike ImuManager. This
    // layer is ESP-IDF-only and genuinely needs what only Device offers --
    // claimOwnership(), healthy(), address() -- and asking for the concrete
    // type is more honest than downcasting a Bus* and hoping. The PURE layer
    // still sees only i2c::Bus&, which is what keeps the host tests running.
    //
    // Configures the pins and builds the driver. Does not create the task and
    // does not touch the chip.
    esp_err_t init(i2c::Device& device, const Wiring& wiring, const Behavior& behavior,
                   const Geometry& geometry);

    // Creates the task, resets and configures the chip, THEN registers the ISR.
    // That order is not cosmetic: an interrupt arriving before the task exists
    // has nowhere to go, and one arriving mid-configuration reads a chip whose
    // registers are half written.
    esp_err_t start();

    // Reverses it exactly: remove the ISR first, so a late edge cannot reach a
    // task or a semaphore that is being freed, then stop the task, then release
    // everything. Also forces a release into the queue -- a UI must never be
    // left holding a widget down because the driver went away.
    void stop();

    bool  isRunning() const { return started_; }
    State state() const { return state_.load(std::memory_order_relaxed); }

    // ---------------------------------------------------------------- for UI

    // Oldest unconsumed transition. Call from the UI task; returns false when
    // the queue is empty.
    //
    // With LVGL 9 this maps straight onto an indev read callback: set
    // lv_indev_data_t from the transition and set `continue_reading` while this
    // keeps returning true, and a tap that happened entirely between two polls
    // still reaches the widget.
    bool popTransition(TouchTransition& out);

    // Current state without consuming anything. Safe from any task.
    TouchSnapshot snapshot() const;

    // ------------------------------------------------------------- commands

    // Routed to the touch task and acknowledged, because the chip has exactly
    // one owner. Returns ESP_ERR_TIMEOUT if the task does not confirm in time.
    //
    // Mode::ChipSleep writes 0xE5 and the chip does NOT come back without a
    // reset pulse; requesting Active again performs that reset.
    esp_err_t requestMode(Mode mode, uint32_t timeoutMs = 500);
    Mode      mode() const { return requestedMode_.load(std::memory_order_relaxed); }

    // Ask for a reset and reconfiguration even though nothing has failed. For
    // the application to call after it has done something that may have
    // disturbed the chip.
    esp_err_t requestRecovery();

    // ---------------------------------------------------------- diagnostics

    void diagnostics(Diagnostics& out) const;
    void logDiagnostics(const char* where) const;

    // Proves the chain end to end and logs one PASS/FAIL line per check.
    // Register work happens ON THE TOUCH TASK, never on the caller's: this
    // device has one owner and i2c::Device says so out loud when that is
    // violated. Must be called after start().
    //
    // waitForTouchMs > 0 asks the operator to touch the screen and reports
    // what arrived; 0 skips that step.
    esp_err_t runSelfTest(SelfTestResult& out, uint32_t waitForTouchMs = 0);

private:
    static constexpr uint32_t kStopTimeoutMs = 1000;
    static constexpr uint32_t kLockTimeoutMs = 50;

    // Consecutive idle ticks with INT asserted and no new edge before the chip
    // is reset. At the default idleTickMs of 40 ms that is ~200 ms of a line
    // that is going nowhere -- long enough that no ordinary pulse can reach it,
    // short enough that a real stuck line does not lock touch out for a second.
    static constexpr uint8_t kIntStuckTicksBeforeRecovery = 5;

    static void IRAM_ATTR isrHandler(void* arg);
    static void           taskFunc(void* arg);

    void run();
    void serviceInterrupts(uint32_t nowMs);
    void checkInterruptStuck();
    void handleFrame(const ParsedFrame& frame, uint32_t nowMs, const uint8_t* raw,
                     uint32_t latencyUs);
    void countFrame(const ParsedFrame& frame);

    // Everything that touches the chip. Touch task only.
    esp_err_t resetAndConfigure(bool logResult);
    void      pulseReset();
    void      enterRecovery(const char* why, uint32_t nowMs);
    void      applyMode(uint32_t nowMs);

    void setState(State s);
    bool intActive() const;
    void maskIrq();
    void unmaskIrq();

    bool lock() const;
    void unlock() const;

    void haltTask();
    void releaseResources();
    void performSelfTest(uint32_t waitForTouchMs);

    i2c::Device* device_{nullptr};
    Wiring       wiring_{};
    Behavior     behavior_{};
    Geometry     geometry_{};

    // Cst816t holds a reference and has no default constructor, so it is built
    // in init() once the bus is known. A raw buffer keeps it off the heap.
    alignas(Cst816t) unsigned char devStorage_[sizeof(Cst816t)]{};
    Cst816t* chip_{nullptr};

    TaskHandle_t      taskHandle_{nullptr};
    SemaphoreHandle_t mutex_{nullptr};
    SemaphoreHandle_t stopSemaphore_{nullptr};
    SemaphoreHandle_t commandDone_{nullptr};
    SemaphoreHandle_t selfTestDone_{nullptr};

    // Guarded by mutex_. A real FreeRTOS mutex, not the binary semaphore the
    // IDF I2C driver uses -- so it DOES carry priority inheritance, and the UI
    // task cannot wedge the touch task the way section 8.1 describes.
    TouchTracker tracker_{};

    std::atomic<State>    state_{State::Stopped};
    std::atomic<Mode>     requestedMode_{Mode::Active};
    std::atomic<bool>     stopRequested_{false};
    std::atomic<bool>     recoveryRequested_{false};
    std::atomic<bool>     selfTestRequested_{false};

    // Written by the ISR, read by the task.
    std::atomic<uint32_t> irqCount_{0};
    std::atomic<int64_t>  lastIrqUs_{0};

    uint32_t servicedIrq_{0};
    Mode     appliedMode_{Mode::Active};

    uint8_t  consecutiveReadFailures_{0};
    uint8_t  intStuckTicks_{0};
    uint8_t  recoveryAttempts_{0};
    uint32_t faultUntilMs_{0};
    uint32_t lastContactIrqMs_{0};

    // Frame log rate limiter.
    uint32_t logWindowStartMs_{0};
    uint32_t logWindowCount_{0};

    mutable Diagnostics diag_{};
    uint64_t            irqToReadTotalUs_{0};
    uint32_t            irqToReadSamples_{0};
    uint64_t            contactIrqTotalMs_{0};
    uint32_t            contactIrqSamples_{0};

    SelfTestResult selfTestResult_{};
    uint32_t       selfTestWaitMs_{0};

    bool started_{false};
    bool isrServiceOwned_{false};
    bool isrRegistered_{false};
};

}  // namespace touch
