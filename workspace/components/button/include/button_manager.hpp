// ESP-IDF integration layer: GPIO, ISR, task, queue.
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

struct ButtonEventMsg {
    gpio_num_t  pin{};
    ButtonEvent event{ButtonEvent::NONE};
    uint32_t    timestampMs{};       // when the FSM EMITTED the event
    uint32_t    pressTimestampMs{};  // when the press edge STARTED the gesture
};

class ButtonManager {
public:
    struct Config {
        uint32_t    pollIntervalMs{kDefaultPollIntervalMs};
        uint32_t    taskStackSize{3072};
        UBaseType_t taskPriority{10};
        BaseType_t  taskCoreId{tskNO_AFFINITY};
        uint8_t     queueLength{10};
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
    esp_err_t addButton(const ButtonConfig& config);

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

    static void IRAM_ATTR isrHandler(void* arg);
    static void           taskFunc(void* arg);

    void      run();
    bool      pollOnce(uint32_t nowMs);
    esp_err_t configureGpio(const ButtonConfig& config);

    // Stop handshake (§10.3). Returns true when the task exited cleanly.
    bool haltTask();
    // Handshake then release, honouring the timeout policy of §10.3.
    void teardownTask();
    // Delete queue + semaphore if still present. Safe to call repeatedly (§10.5).
    void releaseResources();

    Config              config_;
    std::vector<Button> buttons_;

    TaskHandle_t      taskHandle_{nullptr};
    QueueHandle_t     eventQueue_{nullptr};
    SemaphoreHandle_t stopSemaphore_{nullptr};

    std::atomic<bool> stopRequested_{false};
    bool              started_{false};
    bool              isrServiceOwned_{false};
};

}  // namespace button
