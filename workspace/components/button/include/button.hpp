// Pure logic layer: debounce + FSM.
// No dependency on GPIO or FreeRTOS, so it compiles on a PC with g++.
// See doc-design/button-esp-idf-design.md sections 5, 8, 9.
#pragma once

#include <cstdint>

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#else
// Host build fallback: just enough to represent a pin number.
using gpio_num_t = int;
#endif

namespace button {

// ------------------------------------------------------------------ data types

enum class ButtonState : uint8_t {
    IDLE,               // released, nothing pending
    PRESSED,            // held down, not yet known if click or long press
    WAIT_DOUBLE_CLICK,  // released, waiting to see if a second click arrives
    WAIT_RELEASE_LONG,  // LONG_PRESS already emitted, waiting for release
    WAIT_RELEASE,       // DOUBLE_CLICK already emitted, wait for release silently
};

enum class ButtonEvent : uint8_t {
    NONE,
    PRESS_DOWN,  // the debounced press edge itself, before any gesture is known
    CLICK,
    DOUBLE_CLICK,
    LONG_PRESS,
    LONG_PRESS_RELEASE,
};

struct ButtonConfig {
    gpio_num_t pin{};
    bool       activeLow{true};
    bool       enableInternalPull{true};
    bool       enableDoubleClick{true};

    // Off by default, because turning it on inserts an extra event in front of
    // every gesture and would change the sequence an existing app receives.
    //
    // Turn it on when something must react at the instant of the press -- a
    // feedback beep, a haptic tick, a screen waking up. Those cannot wait for
    // CLICK: with enableDoubleClick on, CLICK is held back by doubleClickMs
    // while the FSM rules out a second press (§9.2), which is a delay a finger
    // can feel. PRESS_DOWN reports the pin, not a gesture, so nothing is pending
    // and it goes out as soon as the debounce integrator settles.
    bool       enablePressDown{false};

    uint32_t   longPressMs{800};
    uint32_t   doubleClickMs{250};
    uint32_t   debounceMs{20};   // real time; the cycle count is derived from it
};

// ------------------------------------------------- tick-rate independence (§5)

#ifdef ESP_PLATFORM

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

#endif  // ESP_PLATFORM

// Convert debounceMs into a number of poll cycles (ceil), clamped to [1, 255].
// Free function, pure arithmetic, so host tests can call it directly.
constexpr uint8_t debounceCountFor(uint32_t debounceMs, uint32_t pollIntervalMs)
{
    if (pollIntervalMs == 0) {
        return 1;
    }
    uint32_t n = (debounceMs + pollIntervalMs - 1) / pollIntervalMs;  // ceil
    if (n == 0) {
        n = 1;
    }
    if (n > 255) {
        n = 255;
    }
    return static_cast<uint8_t>(n);
}

// ---------------------------------------------------------------------- Button

class Button {
public:
    // debounceMaxCnt == 0 is forced to 1: with 0 the integrator's two thresholds
    // collapse onto each other and stableState_ latches at one value forever
    // (see section 8 of the design document).
    Button(const ButtonConfig& config, uint8_t debounceMaxCnt);

    // Call once per poll cycle. rawPressed is already normalised for activeLow
    // (true means pressed). Returns ONE event, or NONE.
    // Precedence rule: edges first, timeouts second (§9.2).
    ButtonEvent update(bool rawPressed, uint32_t nowMs);

    bool        isPressed() const { return stableState_; }
    ButtonState getState()  const { return fsmState_; }
    gpio_num_t  pin()       const { return config_.pin; }

    const ButtonConfig& config() const { return config_; }

    // Timestamp of the press edge that STARTED the current gesture. It is never
    // updated mid-gesture, so for DOUBLE_CLICK it still refers to the first
    // press (§4.4).
    uint32_t pressTimestampMs() const { return pressTimestamp_; }

    // The debounceCounter_ == 0 term is mandatory: checking fsmState_ alone lets
    // the manager fall asleep while the integrator is still climbing, and by then
    // the pin has settled so no further edge will ever wake it.
    bool isIdle() const
    {
        return fsmState_ == ButtonState::IDLE && debounceCounter_ == 0;
    }

    void reset();

private:
    void updateDebounce(bool rawPressed);

    ButtonConfig config_;
    uint8_t      debounceMaxCnt_;

    uint8_t     debounceCounter_{0};
    bool        stableState_{false};
    bool        prevStable_{false};
    ButtonState fsmState_{ButtonState::IDLE};
    uint32_t    pressTimestamp_{0};
    uint32_t    clickTimestamp_{0};
};

}  // namespace button
