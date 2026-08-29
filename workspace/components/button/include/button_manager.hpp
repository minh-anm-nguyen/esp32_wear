// ESP-IDF integration layer: GPIO, ISR, task, queue.
//
// Everything platform-shaped lives here, including the tick-rate helpers that
// used to sit in button.hpp behind an #ifdef. button.hpp is now unconditionally
// portable; this header is unconditionally not.
//
// See doc-design/button-esp-idf-design.md sections 7.3, 10, 11.
#pragma once

#include "button.hpp"

#include <atomic>
#include <vector>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace button {

// ------------------------------------------------------------- how it is wired

// The hardware half of what used to be one ButtonConfig. The logic layer never
// needed any of it, and carrying it there is what forced button.hpp to fake a
// gpio_num_t for host builds.
struct ButtonWiring {
    gpio_num_t pin{};
    bool       activeLow{true};
    bool       enableInternalPull{true};

    // Make this button wake the chip from light sleep.
    //
    // The ANYEDGE interrupt configured by the manager is NOT a wake source: it
    // only works while the CPU is running. Waking needs gpio_wakeup_enable(),
    // and that call accepts LEVEL triggers ONLY -- no edges. So a wake-capable
    // button carries two configurations at once: ANYEDGE for the running
    // interrupt, and the pressed LEVEL for the sleep path.
    //
    // Off by default because it costs a wake source on a pin the application
    // may not want waking the watch, and because the level is derived from
    // activeLow -- a wrongly wired button would then hold the chip awake.
    bool       enableWakeup{false};
};

// ------------------------------------------------- tick-rate independence (§5)

inline constexpr uint32_t kMinPreferredPollMs = 5;

// Layer 0: the default adapts itself. HZ=100 -> 10ms; HZ=1000 -> 5ms.
inline constexpr uint32_t kDefaultPollIntervalMs =
    (portTICK_PERIOD_MS > kMinPreferredPollMs)
        ? static_cast<uint32_t>(portTICK_PERIOD_MS)
        : kMinPreferredPollMs;

static_assert(pdMS_TO_TICKS(kDefaultPollIntervalMs) > 0,
              "kDefaultPollIntervalMs truncates to 0 FreeRTOS ticks");

namespace detail {
// Declared but never defined, and deliberately not constexpr. Calling it from a
// consteval context makes the constant expression fail, and the compiler quotes
// this name back. The function name IS the error message.
uint32_t pollIntervalMs_is_below_one_FreeRTOS_tick__raise_it_or_raise_CONFIG_FREERTOS_HZ();
}  // namespace detail

// Layer 1: reject a bad literal supplied by the caller, at compile time.
//   cfg.pollIntervalMs = button::PollMs(5);   // HZ=100 -> compile error
//
// Do NOT use 'throw' here: ESP-IDF builds with -fno-exceptions, and GCC rejects
// the throw keyword during parsing, even inside a branch that only ever runs at
// compile time.
consteval uint32_t PollMs(uint32_t ms)
{
    if (pdMS_TO_TICKS(ms) == 0) {
        detail::pollIntervalMs_is_below_one_FreeRTOS_tick__raise_it_or_raise_CONFIG_FREERTOS_HZ();
    }
    return ms;
}

// ------------------------------------------------------------------- event msg

struct ButtonEventMsg {
    gpio_num_t  pin{};
    ButtonEvent event{ButtonEvent::NONE};
    uint32_t    timestampMs{};       // when the FSM EMITTED the event
    uint32_t    pressTimestampMs{};  // when the press edge STARTED the gesture
};

// ---------------------------------------------------------------- ButtonManager

class ButtonManager {
public:
    struct Config {
        uint32_t    pollIntervalMs{kDefaultPollIntervalMs};
        uint32_t    taskStackSize{3072};

        // 9, deliberately BELOW the tasks that share the I2C bus (touch 11,
        // IMU 10).
        //
        // The IDF I2C driver serialises transactions with a binary semaphore,
        // and a binary semaphore has NO priority inheritance. A task sitting at
        // a priority between two I2C users can therefore preempt the one
        // holding the bus and block the other one indefinitely -- even though
        // it never touches I2C itself. At 10 this task was exactly that wedge
        // between touch and the IMU.
        //
        // See doc-design/i2c-bus-design.md sections 8.1 and 11.1.
        UBaseType_t taskPriority{9};

        BaseType_t  taskCoreId{tskNO_AFFINITY};
        uint8_t     queueLength{10};

        // Flags for gpio_install_isr_service(), used ONLY if no other component
        // has installed it yet. The service is a system-wide singleton and the
        // first caller's flags win for everyone, so leaving this to whichever
        // component happens to start first makes the flags an accident of
        // initialisation order.
        //
        // Prefer installing the service once from app_main and letting every
        // manager fall through the ESP_ERR_INVALID_STATE branch. This field
        // exists so the fallback path is at least explicit.
        //
        // ESP_INTR_FLAG_IRAM is the one that matters here: without it no GPIO
        // interrupt is serviced while the cache is disabled, i.e. during every
        // flash write. Adding it obliges EVERY handler on the shared service to
        // be IRAM-safe.
        int         isrFlags{0};
    };

    // Two separate constructors instead of one with a '= {}' default argument:
    // a nested class's default member initializers are not usable anywhere
    // inside the enclosing class body, so '= {}' there is a compile error.
    // The definitions live in the .cpp, where Config is complete.
    ButtonManager();
    explicit ButtonManager(const Config& config);
    ~ButtonManager();

    // This class owns a task, a queue and ISR handlers, so it must not be copied.
    ButtonManager(const ButtonManager&)            = delete;
    ButtonManager& operator=(const ButtonManager&) = delete;

    // Only valid in the CONFIGURING state (before start, or after stop).
    //
    // Two overloads rather than a defaulted ButtonBehavior argument, matching
    // the constructor pair above: the wiring is what every caller must state,
    // the gesture feel has a sensible default.
    esp_err_t addButton(const ButtonWiring& wiring);
    esp_err_t addButton(const ButtonWiring& wiring, const ButtonBehavior& behavior);

    esp_err_t start();
    void      stop();

    bool isRunning() const { return started_; }

    // Returns nullptr before start().
    QueueHandle_t eventQueue() const { return eventQueue_; }

    // timeoutMs == UINT32_MAX means wait forever.
    // Note: a non-zero timeoutMs smaller than portTICK_PERIOD_MS truncates to
    // 0 ticks, i.e. no waiting at all.
    bool waitEvent(ButtonEventMsg& out, uint32_t timeoutMs);

private:
    static constexpr uint32_t kStopTimeoutMs = 1000;

    // The wiring and the FSM are separate types now, so the manager is the one
    // place that pairs them up again. It is also the only place that needs to.
    struct Entry {
        ButtonWiring wiring;
        Button       logic;
    };

    static void IRAM_ATTR isrHandler(void* arg);
    static void           taskFunc(void* arg);

    void      run();
    bool      pollOnce(uint32_t nowMs);
    esp_err_t configureGpio(const ButtonWiring& wiring);

    // Stop handshake (§10.3). Returns true when the task exited cleanly.
    bool haltTask();
    // Handshake then release, honouring the timeout policy of §10.3.
    void teardownTask();
    // Delete queue + semaphore if still present. Safe to call repeatedly (§10.5).
    void releaseResources();

    Config             config_;
    std::vector<Entry> buttons_;

    TaskHandle_t      taskHandle_{nullptr};
    QueueHandle_t     eventQueue_{nullptr};
    SemaphoreHandle_t stopSemaphore_{nullptr};

    std::atomic<bool> stopRequested_{false};
    bool              started_{false};
    bool              isrServiceOwned_{false};
};

}  // namespace button
