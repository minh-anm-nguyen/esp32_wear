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
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace buzzer {

// The hardware half of what used to be one BuzzerConfig. The sequencer never
// read either field; carrying them in buzzer.hpp is what forced that header to
// fake a gpio_num_t for host builds.
struct BuzzerWiring {
    gpio_num_t pin{};
    bool       activeLow{false};  // true when the driver stage inverts the pin
};

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

        // freq * 2^resolution must stay under the source clock. 10 bit on APB
        // allows ~78 kHz, 13 bit only ~9.7 kHz. start() checks it against
        // maxFreqHz rather than letting high notes come out silently wrong.
        ledc_timer_bit_t resolution{LEDC_TIMER_10_BIT};

        // LEDC_AUTO_CLK usually lands on APB, and on the ESP32-S3 the APB clock
        // follows CPU_CLK. With esp_pm dynamic frequency scaling that means the
        // divider computed by ledc_set_freq() silently goes wrong and notes come
        // out at the wrong pitch -- the LEDC driver holds no PM lock of its own.
        //
        // This manager solves that by holding an ESP_PM_APB_FREQ_MAX lock for
        // exactly as long as a pattern is playing (see the note on sleepMode),
        // so the default stays on the accurate clock.
        //
        // Switch to LEDC_USE_RC_FAST_CLK only together with sleepMode =
        // LEDC_SLEEP_MODE_KEEP_ALIVE, and read that comment first.
        ledc_clk_cfg_t clockSource{LEDC_AUTO_CLK};

        // What the LEDC channel does when the system enters light sleep.
        //
        // The default, NO_ALIVE_NO_PD, means the output STOPS during light
        // sleep. On its own that would chop a note in half, because the buzzer
        // task spends the length of every note blocked on its queue and the
        // system is free to sleep in that window. The PM lock is what closes
        // that hole: while a pattern is playing the system cannot light sleep at
        // all, so the output never stops.
        //
        // LEDC_SLEEP_MODE_KEEP_ALIVE is the alternative -- the tone survives
        // light sleep and no PM lock is needed. It is NOT the default because
        // ledc_channel_config() then pins the timer's clock source powered for
        // EVERY light sleep from that moment on, not just while sounding. On a
        // watch that is a permanent idle-current cost paid for a few seconds of
        // beeping a day. Only worth it for something like a multi-minute alarm
        // that must keep sounding while the rest of the system sleeps.
        ledc_sleep_mode_t sleepMode{LEDC_SLEEP_MODE_NO_ALIVE_NO_PD};

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
    //
    // Two arguments because they answer two different questions: how the device
    // is wired to this board, and what the device is physically able to do.
    esp_err_t init(const BuzzerWiring& wiring, const BuzzerSpec& spec);

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

    // Latch the pad so it keeps its silent level through DEEP sleep.
    //
    // On the ESP32/S2/C3/S3 a digital pad is reset to its default state on deep
    // sleep entry no matter what, unless the pad is held AND
    // gpio_deep_sleep_hold_en() has been called. A floating pad on a transistor
    // driver stage can leave the buzzer half on: no sound, but real current --
    // exactly the failure configureHardware() guards against at boot, only now
    // it drains a battery for hours instead of seconds.
    //
    // Must be called AFTER stop(): it touches the same hardware the buzzer task
    // owns, and this class does not share a device between two tasks. Returns
    // ESP_ERR_INVALID_STATE if the task is still running.
    //
    // The caller still has to invoke gpio_deep_sleep_hold_en() once, and
    // gpio_hold_dis() on the way back if the buzzer is used again.
    esp_err_t holdPinThroughDeepSleep();

private:
    static constexpr uint32_t kStopTimeoutMs = 1000;

    // The ESP32-S3 has no high-speed LEDC mode. Naming it keeps the port to a
    // chip that does have one down to a single line.
    static constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;

    // Reference for the resolution check, derived from Config::clockSource --
    // it is NOT a constant any more: RC_FAST is ~17.5 MHz, not 80.
    uint32_t srcClkHz() const;

    // Hold ESP_PM_APB_FREQ_MAX while, and only while, a pattern is sounding.
    // Idempotent: esp_pm locks count, so a bare acquire/release pair per note
    // boundary would drift out of balance.
    void setPmLock(bool want);

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
    BuzzerWiring          wiring_{};  // meaningful only once buzzer_ is set
    std::optional<Buzzer> buzzer_;    // empty until init()

    TaskHandle_t      taskHandle_{nullptr};
    QueueHandle_t     cmdQueue_{nullptr};
    SemaphoreHandle_t stopSemaphore_{nullptr};

    // Null when CONFIG_PM_ENABLE is off: esp_pm_lock_create() then returns
    // ESP_ERR_NOT_SUPPORTED, which is not an error here, just "nothing to do".
    esp_pm_lock_handle_t pmLock_{nullptr};

    std::atomic<bool> stopRequested_{false};
    bool              started_{false};
    bool              muted_{false};     // only ever read or written by the buzzer task
    bool              pmLockHeld_{false};  // likewise
};

}  // namespace buzzer
