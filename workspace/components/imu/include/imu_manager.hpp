// ESP-IDF integration layer for the QMI8658C: GPIO interrupt, one task, one
// event queue, and a self-test that proves the whole chain works using nothing
// but the chip's own registers and a clock.
//
// See doc-design/imu-qmi8658c-design.md sections 11 and 15.
#pragma once

#include "axis_remap.hpp"
#include "qmi8658.hpp"

#include "i2c_bus.hpp"
#include "i2c_device.hpp"
#include "sensor_types.hpp"

#include <atomic>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace imu {

// -------------------------------------------------------------------- events

struct ImuEventMsg {
    // ONE interrupt can produce SEVERAL of these: STATUS1 is read-to-clear, so
    // the task decodes every flag from a single read and posts them in turn.
    enum class Type : uint8_t {
        WAKE_ON_MOTION,
        STEP,
        TAP,
        ANY_MOTION,
        NO_MOTION,
        SIG_MOTION,
        BUS_ERROR,
    };

    Type      type{Type::BUS_ERROR};
    uint32_t  timestampMs{};
    uint32_t  stepCount{};          // only meaningful for STEP
    esp_err_t err{ESP_OK};          // only meaningful for BUS_ERROR
};

// ------------------------------------------------------------------ self test

// Every field is something the firmware established about itself. No meter, no
// scope: see section 15 of the design document.
struct SelfTestResult {
    bool  identityOk{false};        // WHO_AM_I + REVISION_ID in one burst
    bool  autoIncrementOk{false};   // ...which also proves CTRL1.ADDR_AI
    bool  resetOk{false};           // 0x4D read back 0x80
    bool  handshakeOk{false};       // a CTRL9 NOP completed and was ACKed
    bool  modeReadbackOk{false};    // CTRL2/CTRL7 read back what we wrote
    bool  odrOk{false};             // counted interrupts ~= declared ODR
    bool  gravityOk{false};         // |a| within 5% of 1 g while still

    float measuredOdrHz{0.0f};
    float measuredG{0.0f};

    bool allPassed() const
    {
        return identityOk && autoIncrementOk && resetOk && handshakeOk &&
               modeReadbackOk && odrOk && gravityOk;
    }
};

// ------------------------------------------------------------------ ImuManager

class ImuManager {
public:
    struct Config {
        // The application owns the bus and the Device; this class only borrows.
        i2c::Bus* bus{nullptr};

        gpio_num_t intPin{GPIO_NUM_38};  // QMI_INT2 on V2.1. INT1 is not routed.

        // Enable the ESP32 internal pull-up on the interrupt pin. Harmless for a
        // push-pull output; required for an open-drain one. The datasheet does
        // not say which the QMI8658C uses.
        bool intPullUp{true};

        AxisRemap remap{};

        // Where the sample stream goes. Null is fine -- the application can
        // simply poll latest(). This manager never learns what implements it,
        // which is what keeps components/imu independent of components/motion.
        sensors::ISampleSink* sink{nullptr};

        AccelRange accelRange{AccelRange::G4};
        GyroRange  gyroRange{GyroRange::DPS256};
        WomConfig  wom{};

        uint32_t    taskStackSize{4096};

        // 10, immediately BELOW touch (11) and adjacent to it. The IDF I2C bus
        // lock is a binary semaphore with no priority inheritance, so every task
        // that does NOT use I2C must stay outside the 10..11 band or it can
        // block touch indirectly through this one. See i2c-bus-design.md 8.1.
        UBaseType_t taskPriority{10};
        BaseType_t  taskCoreId{tskNO_AFFINITY};
        uint8_t     queueLength{8};

        // The GPIO ISR service is a system-wide singleton. Calling
        // gpio_install_isr_service() when it already exists logs an ERROR
        // from inside IDF before returning ESP_ERR_INVALID_STATE -- tolerating
        // the return value does not silence the noise. Set false when
        // app_main has installed it, which is the arrangement that also
        // stops the flags being an accident of initialisation order.
        bool installIsrService{true};

        // CTRL1 bit 4. Rev A calls bits 4:3 "Reserved"; SensorLib drives them
        // as the INT1/INT2 output enables and works on this hardware. With it
        // clear, GPIO38 never moves and the ODR self-test counts zero.
        // Section 19 flagged this as the thing to try if INT2 stays silent --
        // and on this board it stayed silent.
        bool enableInt2OutputBit{true};
    };

    ImuManager() = default;
    ~ImuManager();

    ImuManager(const ImuManager&)            = delete;
    ImuManager& operator=(const ImuManager&) = delete;

    // Runs the mandatory bring-up sequence of section 13, in order. Does not
    // create the task.
    esp_err_t init(const Config& config);

    esp_err_t start();
    void      stop();

    bool isRunning() const { return started_; }

    // Callable from ANY task: it only records a request and wakes the IMU task,
    // which owns the Qmi8658 exclusively. That ownership is the reason there is
    // no mutex in this class -- and i2c::Device enforces it at runtime.
    esp_err_t setPowerMode(PowerMode mode);
    PowerMode powerMode() const { return requestedMode_.load(); }

    // The real frequency in the current mode. Not a constant: see section 4.2.
    float accelOdrHz() const;

    // Latest sample, already converted and remapped. Lock free (seqlock), so a
    // reader never delays the sensor task.
    bool latest(sensors::Sample& out) const;

    bool waitEvent(ImuEventMsg& out, uint32_t timeoutMs);

    uint32_t stepCount() const { return stepCount_.load(); }

    // The remap currently being applied to every published Sample.
    // runAxisCalibration() needs this: it can only derive a mapping from
    // SENSOR-frame readings, so it refuses to run once a remap is in place.
    const AxisRemap& remap() const { return config_.remap; }

    // Proves the chain end to end and logs one PASS/FAIL line per check.
    // Must be called after start(). Takes a little over a second: it counts
    // data-ready interrupts for exactly 1000 ms to derive the true ODR.
    // Register access happens ON THE IMU TASK, not the caller's: this device
    // has one owner, and i2c::Device says so out loud when that is violated.
    // The caller only counts interrupts and reads latest(), both of which
    // are already safe to touch from anywhere.
    esp_err_t runSelfTest(SelfTestResult& out);

private:
    static constexpr uint32_t kStopTimeoutMs = 1000;

    // A missed edge must not wedge the task forever, so the wait always has a
    // ceiling. In ACTIVITY a full second of silence means data-ready stopped.
    static constexpr uint32_t kWatchdogMs = 1000;

    static void IRAM_ATTR isrHandler(void* arg);
    static void           taskFunc(void* arg);

    void run();
    void serviceInterrupt();
    void publish(const RawSample& raw, uint32_t nowMs);
    void post(ImuEventMsg::Type type, uint32_t nowMs, uint32_t steps = 0,
              esp_err_t err = ESP_OK);

    bool applyPendingMode();

    // Phase A of the self-test: everything that talks to the chip. Runs on
    // the IMU task, triggered by runSelfTest() and reported back through
    // selfTestDone_.
    void performRegisterSelfTest();

    // Settles which physical pin GPIO38 is bonded to by routing the CTRL9
    // handshake to INT1 and watching the pad. Runs only when the chip says
    // it is asserting INT2 while the pad stays still.
    void probeWhichIntPin();

    bool haltTask();
    void teardownTask();
    void releaseResources();

    Config config_{};
    // Qmi8658 has no default constructor and holds a reference, so it is built
    // in init() once the bus is known. A raw buffer keeps it out of the heap.
    alignas(Qmi8658) unsigned char devStorage_[sizeof(Qmi8658)]{};
    Qmi8658* dev_{nullptr};

    TaskHandle_t      taskHandle_{nullptr};
    QueueHandle_t     eventQueue_{nullptr};
    SemaphoreHandle_t stopSemaphore_{nullptr};

    std::atomic<bool>      stopRequested_{false};
    std::atomic<PowerMode> requestedMode_{PowerMode::OFF};
    std::atomic<uint32_t>  stepCount_{0};

    // Counts data-ready services. runSelfTest() samples it over a known window
    // to derive the ODR that is actually happening, which is what stands in for
    // a frequency meter.
    std::atomic<uint32_t> sampleCount_{0};

    // Seqlock: writer bumps to odd, writes, bumps to even. A reader that sees an
    // odd or changed counter simply retries. 28 bytes cannot be written
    // atomically, and a mutex here would let any reader stall the sensor task.
    std::atomic<uint32_t> snapshotSeq_{0};
    sensors::Sample       snapshot_{};

    // Self-test hand-off. The caller parks on selfTestDone_ while the task
    // does the register work in its own context.
    std::atomic<bool> selfTestRequested_{false};
    SemaphoreHandle_t selfTestDone_{nullptr};
    SelfTestResult    selfTestPartial_{};

    // One-shot report of whether samples arrive by interrupt or by polling.
    bool     pathReported_{false};
    uint32_t pathSamples_{0};

    PowerMode appliedMode_{PowerMode::OFF};
    bool      started_{false};
    bool      isrServiceOwned_{false};
};

// ------------------------------------------------------- axis calibration

// Walks the user through three orientations and derives the AxisRemap.
//
// Nothing is measured by hand: the firmware prints a prompt, waits until
// the watch has been still for a moment, averages what it sees, and at the
// end logs a ready-to-paste AxisRemap literal. The operator only turns the
// watch when asked.
//
// The IMU must already be started and in a mode that produces samples
// (ACTIVITY). Returns ESP_ERR_TIMEOUT if a pose is never held still.
esp_err_t runAxisCalibration(ImuManager& manager, AxisRemap& out);

}  // namespace imu
