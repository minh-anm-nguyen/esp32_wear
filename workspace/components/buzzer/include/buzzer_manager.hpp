// ESP-IDF integration layer: LEDC (or plain GPIO), one task, one command queue.
//
// Mirror of button_manager.hpp with the queue pointing the other way: the
// application SENDS commands here, it does not receive events. There is no ISR,
// because nothing outside can change a buzzer's state on its own.
#pragma once

#include "buzzer.hpp"

#include <atomic>
#include <optional>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace buzzer {

struct BuzzerCommand {
    enum class Type : uint8_t { PLAY, STOP, MUTE, UNMUTE };

    Type    type{Type::STOP};
    Pattern pattern{};  // only meaningful for PLAY
};

class BuzzerManager {
public:
    struct Config {
        // A LEDC timer is shared by every channel bound to it, and ledc_set_freq()
        // retunes the TIMER. Give the buzzer a timer of its own: sharing one with
        // the display backlight means every note flickers the screen.
        ledc_timer_t     timer{LEDC_TIMER_0};
        ledc_channel_t   channel{LEDC_CHANNEL_0};

        // freq * 2^resolution must stay under the source clock (80 MHz). 10 bit
        // allows ~78 kHz, 13 bit only ~9.7 kHz. start() checks it against
        // maxFreqHz rather than letting high notes come out silently wrong.
        ledc_timer_bit_t resolution{LEDC_TIMER_10_BIT};

        uint32_t    taskStackSize{3072};
        UBaseType_t taskPriority{5};  // below the button task: a late beep is harmless
        BaseType_t  taskCoreId{tskNO_AFFINITY};
        uint8_t     queueLength{4};
    };

    // Two constructors rather than a '= {}' default argument, for the same reason
    // as ButtonManager: a nested class's default member initializers are not
    // usable anywhere inside the enclosing class body.
    BuzzerManager();
    explicit BuzzerManager(const Config& config);
    ~BuzzerManager();

    // Owns a task, a queue and a LEDC channel, so it must not be copied.
    BuzzerManager(const BuzzerManager&)            = delete;
    BuzzerManager& operator=(const BuzzerManager&) = delete;

    // One device per manager, unlike ButtonManager::addButton(). A second buzzer
    // needs its own LEDC timer anyway, so sharing a task would buy nothing.
    // Only valid before start().
    esp_err_t init(const BuzzerConfig& config);

    esp_err_t start();
    void      stop();

    bool isRunning() const { return started_; }

    // Callable from ANY task. None of these touch the sequencer: they only post
    // to the queue, and the buzzer task owns the Buzzer object exclusively. That
    // is the whole reason there is no mutex anywhere in this class.
    //
    // pattern.notes must point at storage that outlives the playback (see the
    // comment on Pattern). The queue copies the Pattern, never the notes.
    esp_err_t play(const Pattern& pattern);
    esp_err_t silence();
    esp_err_t setMuted(bool muted);

private:
    static constexpr uint32_t kStopTimeoutMs = 1000;

    // The ESP32-S3 has no high-speed LEDC mode. Naming it keeps the port to a
    // chip that does have one down to a single line.
    static constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;

    // Reference for the resolution check. LEDC_AUTO_CLK picks APB on this family.
    static constexpr uint32_t kSrcClkHz = 80000000u;

    static void taskFunc(void* arg);
    void        run();

    esp_err_t send(const BuzzerCommand& cmd, TickType_t ticks);
    void      handleCommand(const BuzzerCommand& cmd);

    esp_err_t configureHardware();
    void      applyOutput(const ToneOutput& out);
    void      idleHardware();
    uint32_t  dutyFor(uint8_t volume) const;

    // Stop handshake, same policy as ButtonManager section 10.3.
    bool haltTask();
    void teardownTask();
    void releaseResources();

    Config                config_;
    std::optional<Buzzer> buzzer_;  // empty until init()

    TaskHandle_t      taskHandle_{nullptr};
    QueueHandle_t     cmdQueue_{nullptr};
    SemaphoreHandle_t stopSemaphore_{nullptr};

    std::atomic<bool> stopRequested_{false};
    bool              started_{false};
    bool              muted_{false};  // only ever read or written by the buzzer task
};

}  // namespace buzzer
