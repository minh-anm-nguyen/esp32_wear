#include "button_manager.hpp"

#include <cinttypes>

#include "esp_log.h"
#include "esp_timer.h"

namespace button {

namespace {

constexpr const char* TAG = "button";

// One clock read for the whole poll cycle, so every button sees the SAME instant.
inline uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

}  // namespace

ButtonManager::ButtonManager() : ButtonManager(Config{}) {}

ButtonManager::ButtonManager(const Config& config) : config_(config) {}

ButtonManager::~ButtonManager()
{
    stop();
}

// ----------------------------------------------------------------- configuration

esp_err_t ButtonManager::configureGpio(const ButtonConfig& config)
{
    bool pull = config.enableInternalPull;

    // Input-only pins (34-39 on the original ESP32) have no internal pull
    // resistors. The ESP32-S3 has none of those; this branch exists for
    // portability across the family.
    if (pull && !GPIO_IS_VALID_OUTPUT_GPIO(config.pin)) {
        ESP_LOGW(TAG,
                 "GPIO%d la input-only: khong co tro keo noi bo, can tro ngoai",
                 static_cast<int>(config.pin));
        pull = false;
    }

    gpio_config_t io{};
    io.pin_bit_mask = 1ULL << static_cast<uint32_t>(config.pin);
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = (pull && config.activeLow)  ? GPIO_PULLUP_ENABLE
                                                  : GPIO_PULLUP_DISABLE;
    io.pull_down_en = (pull && !config.activeLow) ? GPIO_PULLDOWN_ENABLE
                                                  : GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_ANYEDGE;

    return gpio_config(&io);
}

esp_err_t ButtonManager::addButton(const ButtonConfig& config)
{
    // Cheap checks first, side effects last.
    if (started_) {
        ESP_LOGE(TAG, "addButton() khong goi duoc khi dang chay");
        return ESP_ERR_INVALID_STATE;
    }

    if (!GPIO_IS_VALID_GPIO(config.pin)) {
        ESP_LOGE(TAG, "GPIO%d khong hop le tren chip nay",
                 static_cast<int>(config.pin));
        return ESP_ERR_INVALID_ARG;
    }

    for (const auto& b : buttons_) {
        if (b.pin() == config.pin) {
            // Two Buttons on one pin means the app receives every press twice.
            ESP_LOGE(TAG, "GPIO%d da duoc them roi",
                     static_cast<int>(config.pin));
            return ESP_ERR_INVALID_ARG;
        }
    }

    const esp_err_t err = configureGpio(config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(GPIO%d) that bai: %s",
                 static_cast<int>(config.pin), esp_err_to_name(err));
        return err;
    }

    buttons_.emplace_back(config,
                          debounceCountFor(config.debounceMs, config_.pollIntervalMs));
    return ESP_OK;
}

// -------------------------------------------------------------------- start / stop

esp_err_t ButtonManager::start()
{
    if (started_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (buttons_.empty()) {
        ESP_LOGE(TAG, "start() can it nhat mot nut");
        return ESP_ERR_INVALID_STATE;
    }

    // Layer 2 of section 5: reject, never silently correct.
    if (pdMS_TO_TICKS(config_.pollIntervalMs) == 0) {
        ESP_LOGE(TAG,
                 "pollIntervalMs=%" PRIu32 " ms < 1 tick (%d ms @ %d Hz). "
                 "Tang pollIntervalMs len >= %d, hoac tang CONFIG_FREERTOS_HZ.",
                 config_.pollIntervalMs, static_cast<int>(portTICK_PERIOD_MS),
                 static_cast<int>(configTICK_RATE_HZ),
                 static_cast<int>(portTICK_PERIOD_MS));
        return ESP_ERR_INVALID_ARG;
    }

    stopRequested_.store(false, std::memory_order_release);
    for (auto& b : buttons_) {
        b.reset();  // do not inherit the FSM state of a previous run
    }

    eventQueue_ = xQueueCreate(config_.queueLength, sizeof(ButtonEventMsg));
    if (eventQueue_ == nullptr) {
        ESP_LOGE(TAG, "khong tao duoc queue");
        releaseResources();
        return ESP_ERR_NO_MEM;
    }

    stopSemaphore_ = xSemaphoreCreateBinary();
    if (stopSemaphore_ == nullptr) {
        ESP_LOGE(TAG, "khong tao duoc semaphore");
        releaseResources();
        return ESP_ERR_NO_MEM;
    }

    // The task MUST exist before the ISR is registered: gpio_isr_handler_add()
    // copies arg by value, so the handle has to be valid at registration time.
    const BaseType_t ok = xTaskCreatePinnedToCore(
        taskFunc, "button", config_.taskStackSize, this,
        config_.taskPriority, &taskHandle_, config_.taskCoreId);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "khong tao duoc task");
        taskHandle_ = nullptr;
        releaseResources();
        return ESP_ERR_NO_MEM;
    }

    // The ISR service is a system-wide resource. Another component having
    // installed it already is not an error.
    esp_err_t err = gpio_install_isr_service(0);
    if (err == ESP_ERR_INVALID_STATE) {
        isrServiceOwned_ = false;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_install_isr_service() that bai: %s", esp_err_to_name(err));
        teardownTask();
        return err;
    } else {
        isrServiceOwned_ = true;
    }

    // Arm the event source LAST: from here on an interrupt may fire at any moment.
    for (size_t i = 0; i < buttons_.size(); ++i) {
        err = gpio_isr_handler_add(buttons_[i].pin(), isrHandler, taskHandle_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "gpio_isr_handler_add(GPIO%d) that bai: %s",
                     static_cast<int>(buttons_[i].pin()), esp_err_to_name(err));
            for (size_t j = 0; j < i; ++j) {
                gpio_isr_handler_remove(buttons_[j].pin());
            }
            teardownTask();
            return err;
        }
    }

    started_ = true;
    return ESP_OK;
}

void ButtonManager::stop()
{
    if (!started_) {
        return;  // guard for the public API; start()'s error path bypasses it
    }

    // Cut the signal source BEFORE destroying what receives it: after this loop
    // no ISR can reference taskHandle_ any more.
    for (auto& b : buttons_) {
        gpio_isr_handler_remove(b.pin());
    }

    teardownTask();

    for (auto& b : buttons_) {
        b.reset();  // stop()/start() must be a clean boundary
    }

    started_ = false;
}

bool ButtonManager::haltTask()
{
    if (taskHandle_ == nullptr) {
        return true;
    }

    // Set the flag BEFORE waking the task. The other order races: the task wakes,
    // reads the flag (still false), polls once, sleeps again -- and the
    // notification has already been consumed.
    stopRequested_.store(true, std::memory_order_release);
    xTaskNotifyGive(taskHandle_);

    if (stopSemaphore_ != nullptr &&
        xSemaphoreTake(stopSemaphore_, pdMS_TO_TICKS(kStopTimeoutMs)) == pdTRUE) {
        return true;
    }
    return false;
}

void ButtonManager::teardownTask()
{
    if (haltTask()) {
        releaseResources();
    } else {
        // On timeout the task may still be alive and still holding pointers to
        // the queue and semaphore. Deleting them now would be a use-after-free.
        // A bounded, logged leak is far safer than a crash at an unknown time.
        ESP_LOGE(TAG,
                 "task khong thoat trong %" PRIu32 " ms: co y ro ri queue + semaphore",
                 kStopTimeoutMs);
        eventQueue_    = nullptr;
        stopSemaphore_ = nullptr;
    }
    taskHandle_ = nullptr;
}

void ButtonManager::releaseResources()
{
    if (eventQueue_ != nullptr) {
        vQueueDelete(eventQueue_);
        eventQueue_ = nullptr;
    }
    if (stopSemaphore_ != nullptr) {
        vSemaphoreDelete(stopSemaphore_);
        stopSemaphore_ = nullptr;
    }
}

// ---------------------------------------------------------------- ISR and task

void IRAM_ATTR ButtonManager::isrHandler(void* arg)
{
    // arg is a TaskHandle_t, not 'this': the ISR dereferences no object at all.
    BaseType_t hpTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(arg), &hpTaskWoken);
    if (hpTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void ButtonManager::taskFunc(void* arg)
{
    static_cast<ButtonManager*>(arg)->run();
}

void ButtonManager::run()
{
    TickType_t       lastWake = xTaskGetTickCount();
    const TickType_t period   = pdMS_TO_TICKS(config_.pollIntervalMs);

    // Start in poll mode so a button already held down at start() is caught.
    bool busy = true;

    while (!stopRequested_.load(std::memory_order_acquire)) {
        if (!busy) {
            // Do NOT clear the notification here. A pending notification is the
            // latch that saves a press landing in the gap between the last
            // pollOnce() and this sleep: ulTaskNotifyTake returns immediately
            // instead of blocking forever.
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if (stopRequested_.load(std::memory_order_acquire)) {
                break;
            }
            // MANDATORY: lastWake may be hours stale. Without this line
            // xTaskDelayUntil spins tick by tick to catch up -> watchdog reset.
            lastWake = xTaskGetTickCount();
        } else {
            ulTaskNotifyTake(pdTRUE, 0);  // minor optimisation: start the cycle clean
            xTaskDelayUntil(&lastWake, period);
        }

        busy = pollOnce(nowMs());
    }

    xSemaphoreGive(stopSemaphore_);
    vTaskDelete(nullptr);
}

bool ButtonManager::pollOnce(uint32_t now)
{
    bool busy = false;

    for (auto& b : buttons_) {
        const int  level      = gpio_get_level(b.pin());
        const bool rawPressed = b.config().activeLow ? (level == 0) : (level != 0);

        const ButtonEvent ev = b.update(rawPressed, now);
        if (ev != ButtonEvent::NONE) {
            const ButtonEventMsg msg{b.pin(), ev, now, b.pressTimestampMs()};
            // Timeout 0: better to drop an event than to block the poll task.
            if (xQueueSend(eventQueue_, &msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "queue day, bo su kien %d cua GPIO%d",
                         static_cast<int>(ev), static_cast<int>(b.pin()));
            }
        }

        if (!b.isIdle()) {
            busy = true;
        }
    }

    return busy;
}

// ----------------------------------------------------------------------- helper

bool ButtonManager::waitEvent(ButtonEventMsg& out, uint32_t timeoutMs)
{
    if (eventQueue_ == nullptr) {
        return false;
    }
    const TickType_t ticks = (timeoutMs == UINT32_MAX) ? portMAX_DELAY
                                                       : pdMS_TO_TICKS(timeoutMs);
    return xQueueReceive(eventQueue_, &out, ticks) == pdTRUE;
}

}  // namespace button
