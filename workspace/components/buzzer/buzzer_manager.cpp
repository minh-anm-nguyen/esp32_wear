#include "buzzer_manager.hpp"

#include <cinttypes>

#include "esp_log.h"
#include "esp_timer.h"

namespace buzzer {

namespace {

constexpr const char* TAG = "buzzer";

inline uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

}  // namespace

BuzzerManager::BuzzerManager() : BuzzerManager(Config{}) {}

BuzzerManager::BuzzerManager(const Config& config) : config_(config) {}

BuzzerManager::~BuzzerManager()
{
    stop();
}

// ----------------------------------------------------------------- configuration

esp_err_t BuzzerManager::init(const BuzzerConfig& config)
{
    if (started_) {
        ESP_LOGE(TAG, "init() khong goi duoc khi dang chay");
        return ESP_ERR_INVALID_STATE;
    }

    // OUTPUT, not merely VALID: an input-only pin can never drive a buzzer.
    if (!GPIO_IS_VALID_OUTPUT_GPIO(config.pin)) {
        ESP_LOGE(TAG, "GPIO%d khong xuat duoc tin hieu tren chip nay",
                 static_cast<int>(config.pin));
        return ESP_ERR_INVALID_ARG;
    }

    if (config.type == BuzzerType::PASSIVE && config.minFreqHz > config.maxFreqHz) {
        ESP_LOGE(TAG, "minFreqHz=%u > maxFreqHz=%u",
                 static_cast<unsigned>(config.minFreqHz),
                 static_cast<unsigned>(config.maxFreqHz));
        return ESP_ERR_INVALID_ARG;
    }

    buzzer_.emplace(config);
    return ESP_OK;
}

esp_err_t BuzzerManager::configureHardware()
{
    const BuzzerConfig& dev = buzzer_->config();

    if (dev.type == BuzzerType::ACTIVE) {
        gpio_config_t io{};
        io.pin_bit_mask = 1ULL << static_cast<uint32_t>(dev.pin);
        io.mode         = GPIO_MODE_OUTPUT;
        io.intr_type    = GPIO_INTR_DISABLE;

        const esp_err_t err = gpio_config(&io);
        if (err != ESP_OK) {
            return err;
        }
        // Park the pin before anything can command a tone.
        return gpio_set_level(dev.pin, dev.activeLow ? 1 : 0);
    }

    // Park the pad at the silent level BEFORE LEDC takes it over. From reset
    // until this line the pad is a floating input, and a floating gate or base
    // resistor on a transistor driver stage can leave the buzzer half on: no
    // sound, but real current and a warm regulator.
    gpio_config_t park{};
    park.pin_bit_mask = 1ULL << static_cast<uint32_t>(dev.pin);
    park.mode         = GPIO_MODE_OUTPUT;
    park.intr_type    = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&park);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_set_level(dev.pin, dev.activeLow ? 1 : 0);
    if (err != ESP_OK) {
        return err;
    }

    ledc_timer_config_t tcfg{};
    tcfg.speed_mode      = kSpeedMode;
    tcfg.duty_resolution = config_.resolution;
    tcfg.timer_num       = config_.timer;
    tcfg.freq_hz         = dev.minFreqHz;  // placeholder, every note retunes it
    tcfg.clk_cfg         = LEDC_AUTO_CLK;

    err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t ccfg{};
    ccfg.gpio_num   = static_cast<int>(dev.pin);
    ccfg.speed_mode = kSpeedMode;
    ccfg.channel    = config_.channel;
    ccfg.timer_sel  = config_.timer;
    ccfg.duty       = 0;  // silent from the very first instant
    ccfg.hpoint     = 0;
    ccfg.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
    // Let the peripheral carry the polarity. Everything downstream then works in
    // terms of duty alone, and duty 0 is the silent level for both wirings.
    ccfg.flags.output_invert = dev.activeLow ? 1u : 0u;

    return ledc_channel_config(&ccfg);
}

// -------------------------------------------------------------------- start / stop

esp_err_t BuzzerManager::start()
{
    if (started_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!buzzer_.has_value()) {
        ESP_LOGE(TAG, "phai goi init() truoc start()");
        return ESP_ERR_INVALID_STATE;
    }

    const BuzzerConfig& dev = buzzer_->config();

    // Reject, never silently correct -- the rule of section 5 of the button
    // design doc. A frequency the resolution cannot express does not fail loudly
    // on its own; it just comes out as the wrong note.
    if (dev.type == BuzzerType::PASSIVE) {
        const uint32_t maxFreq = kSrcClkHz >> static_cast<uint32_t>(config_.resolution);
        if (dev.maxFreqHz > maxFreq) {
            ESP_LOGE(TAG,
                     "do phan giai %d bit chi cho toi %" PRIu32 " Hz, ma maxFreqHz=%u. "
                     "Giam Config::resolution hoac giam maxFreqHz.",
                     static_cast<int>(config_.resolution), maxFreq,
                     static_cast<unsigned>(dev.maxFreqHz));
            return ESP_ERR_INVALID_ARG;
        }
    }

    stopRequested_.store(false, std::memory_order_release);
    muted_ = false;
    buzzer_->reset();  // do not inherit the state of a previous run

    // Hardware FIRST, so the pin is parked silent before the task can exist.
    esp_err_t err = configureHardware();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cau hinh phan cung GPIO%d that bai: %s",
                 static_cast<int>(dev.pin), esp_err_to_name(err));
        return err;
    }

    cmdQueue_ = xQueueCreate(config_.queueLength, sizeof(BuzzerCommand));
    if (cmdQueue_ == nullptr) {
        ESP_LOGE(TAG, "khong tao duoc queue");
        idleHardware();
        return ESP_ERR_NO_MEM;
    }

    stopSemaphore_ = xSemaphoreCreateBinary();
    if (stopSemaphore_ == nullptr) {
        ESP_LOGE(TAG, "khong tao duoc semaphore");
        releaseResources();
        idleHardware();
        return ESP_ERR_NO_MEM;
    }

    const BaseType_t ok = xTaskCreatePinnedToCore(
        taskFunc, "buzzer", config_.taskStackSize, this,
        config_.taskPriority, &taskHandle_, config_.taskCoreId);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "khong tao duoc task");
        taskHandle_ = nullptr;
        releaseResources();
        idleHardware();
        return ESP_ERR_NO_MEM;
    }

    started_ = true;
    return ESP_OK;
}

void BuzzerManager::stop()
{
    if (!started_) {
        return;  // guard for the public API; start()'s error paths bypass it
    }

    teardownTask();

    if (buzzer_.has_value()) {
        buzzer_->reset();  // stop()/start() must be a clean boundary
    }
    started_ = false;
}

bool BuzzerManager::haltTask()
{
    if (taskHandle_ == nullptr) {
        return true;
    }

    // Set the flag BEFORE posting. The other order races: the task wakes, reads
    // the flag (still false), plays one note, sleeps again -- and the wake-up
    // has already been consumed.
    stopRequested_.store(true, std::memory_order_release);

    // ButtonManager wakes its task with a task notification because the ISR
    // already owns that channel. There is no ISR here, so the command queue IS
    // the wake-up channel. SendToFront, not Send: a stop must not queue behind
    // whatever the application posted a moment earlier.
    if (cmdQueue_ != nullptr) {
        const BuzzerCommand wake{BuzzerCommand::Type::STOP, Pattern{}};
        xQueueSendToFront(cmdQueue_, &wake, pdMS_TO_TICKS(kStopTimeoutMs));
    }

    if (stopSemaphore_ != nullptr &&
        xSemaphoreTake(stopSemaphore_, pdMS_TO_TICKS(kStopTimeoutMs)) == pdTRUE) {
        return true;
    }
    return false;
}

void BuzzerManager::teardownTask()
{
    if (haltTask()) {
        releaseResources();
    } else {
        // On timeout the task may still be alive and still holding a pointer to
        // the queue. Deleting it now would be a use-after-free. A bounded, logged
        // leak is far safer than a crash at an unknown time.
        ESP_LOGE(TAG,
                 "task khong thoat trong %" PRIu32 " ms: co y ro ri queue + semaphore",
                 kStopTimeoutMs);
        cmdQueue_      = nullptr;
        stopSemaphore_ = nullptr;

        // The task normally parks the pin on its way out. It did not get there,
        // and a buzzer left sounding is the one failure the user can hear.
        idleHardware();
    }
    taskHandle_ = nullptr;
}

void BuzzerManager::releaseResources()
{
    if (cmdQueue_ != nullptr) {
        vQueueDelete(cmdQueue_);
        cmdQueue_ = nullptr;
    }
    if (stopSemaphore_ != nullptr) {
        vSemaphoreDelete(stopSemaphore_);
        stopSemaphore_ = nullptr;
    }
}

// ------------------------------------------------------------------- public API

esp_err_t BuzzerManager::send(const BuzzerCommand& cmd, TickType_t ticks)
{
    if (!started_ || cmdQueue_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(cmdQueue_, &cmd, ticks) != pdTRUE) {
        // Dropping a beep is a cosmetic loss. Blocking the caller -- which may
        // well be the button task -- is a real one.
        ESP_LOGW(TAG, "queue day, bo lenh %d", static_cast<int>(cmd.type));
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t BuzzerManager::play(const Pattern& pattern)
{
    if (pattern.notes == nullptr || pattern.count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    return send(BuzzerCommand{BuzzerCommand::Type::PLAY, pattern}, 0);
}

esp_err_t BuzzerManager::silence()
{
    return send(BuzzerCommand{BuzzerCommand::Type::STOP, Pattern{}}, 0);
}

esp_err_t BuzzerManager::setMuted(bool muted)
{
    const auto type = muted ? BuzzerCommand::Type::MUTE : BuzzerCommand::Type::UNMUTE;
    return send(BuzzerCommand{type, Pattern{}}, 0);
}

// ------------------------------------------------------------------------- task

void BuzzerManager::taskFunc(void* arg)
{
    static_cast<BuzzerManager*>(arg)->run();
}

void BuzzerManager::handleCommand(const BuzzerCommand& cmd)
{
    switch (cmd.type) {
    case BuzzerCommand::Type::PLAY:
        // May be refused on priority; that is a normal outcome, not an error.
        buzzer_->play(cmd.pattern, nowMs());
        break;

    case BuzzerCommand::Type::STOP:
        buzzer_->stop();
        break;

    case BuzzerCommand::Type::MUTE:
    case BuzzerCommand::Type::UNMUTE:
        muted_ = (cmd.type == BuzzerCommand::Type::MUTE);
        // Re-apply straight away. Without this, muting during a 700 ms alarm note
        // would not be heard until that note happened to end.
        applyOutput(buzzer_->currentOutput());
        break;
    }
}

void BuzzerManager::run()
{
    while (!stopRequested_.load(std::memory_order_acquire)) {
        const uint32_t delayMs = buzzer_->nextDelayMs(nowMs());

        TickType_t ticks;
        if (delayMs == kSleepForever) {
            ticks = portMAX_DELAY;
        } else if (delayMs == 0) {
            ticks = 0;  // overdue: drain the queue without waiting, then act
        } else {
            ticks = pdMS_TO_TICKS(delayMs);
            // Floor of one tick. pdMS_TO_TICKS(4) is 0 at CONFIG_FREERTOS_HZ=100,
            // and a zero-tick receive does not block at all -- this loop would
            // then spin at full speed for the whole note. A note shorter than one
            // tick cannot be timed exactly anyway; round it up rather than burn a
            // core on it.
            if (ticks == 0) {
                ticks = 1;
            }
        }

        // One blocking call covers both jobs: waiting for the current note to end
        // and waiting for the next command. That is the whole reason this task
        // needs no poll interval, no task notification and no timer.
        BuzzerCommand cmd{};
        if (xQueueReceive(cmdQueue_, &cmd, ticks) == pdTRUE) {
            handleCommand(cmd);
        }
        if (stopRequested_.load(std::memory_order_acquire)) {
            break;
        }

        applyOutput(buzzer_->update(nowMs()));
    }

    // Park the pin BEFORE signalling: stop() may free the queue the instant the
    // semaphore is given, and a buzzer left sounding is audible forever.
    idleHardware();
    xSemaphoreGive(stopSemaphore_);
    vTaskDelete(nullptr);
}

// --------------------------------------------------------------------- hardware

uint32_t BuzzerManager::dutyFor(uint8_t volume) const
{
    // A piezo sounds because of the EDGES, not the level: duty 100% is plain DC
    // and is completely silent. Amplitude peaks at 50%, so volume 0..100 maps
    // onto the lower half of the duty range, not the whole of it.
    const uint32_t full = 1u << static_cast<uint32_t>(config_.resolution);
    return (full / 2u) * volume / 100u;
}

void BuzzerManager::applyOutput(const ToneOutput& out)
{
    if (!out.changed) {
        return;  // hold: retouching LEDC would only add a click
    }

    const BuzzerConfig& dev = buzzer_->config();
    const bool          on  = out.on && !muted_;

    if (dev.type == BuzzerType::ACTIVE) {
        gpio_set_level(dev.pin, (on != dev.activeLow) ? 1 : 0);
        return;
    }

    // Close the duty first, on both paths. ledc_set_freq() reloads the timer
    // divider, and doing that while the output is still driving puts an audible
    // click at the head of every note.
    ledc_set_duty(kSpeedMode, config_.channel, 0);
    ledc_update_duty(kSpeedMode, config_.channel);
    if (!on) {
        return;
    }

    const esp_err_t err = ledc_set_freq(kSpeedMode, config_.timer, out.freqHz);
    if (err != ESP_OK) {
        // The divider cannot reach this frequency. Stay silent rather than emit
        // whatever it rounded to: a wrong note is harder to diagnose than none.
        ESP_LOGW(TAG, "khong dat duoc %u Hz: %s",
                 static_cast<unsigned>(out.freqHz), esp_err_to_name(err));
        return;
    }

    ledc_set_duty(kSpeedMode, config_.channel, dutyFor(out.volume));
    ledc_update_duty(kSpeedMode, config_.channel);
}

void BuzzerManager::idleHardware()
{
    if (!buzzer_.has_value()) {
        return;
    }
    const BuzzerConfig& dev = buzzer_->config();

    if (dev.type == BuzzerType::ACTIVE) {
        gpio_set_level(dev.pin, dev.activeLow ? 1 : 0);
        return;
    }

    // ledc_stop(), not merely duty 0: it detaches the PWM and pins the pad at a
    // fixed level, leaving no DC voltage sitting across the piezo. idle_level is
    // 0 because flags.output_invert already carries the polarity.
    ledc_stop(kSpeedMode, config_.channel, 0);
}

}  // namespace buzzer
