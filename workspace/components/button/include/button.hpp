// Pure logic layer: debounce + FSM.
//
// This header includes NOTHING. No GPIO, no FreeRTOS, no ESP-IDF, and no
// #ifdef ESP_PLATFORM shim either -- it compiles on a PC exactly as it compiles
// on the chip, because there is no longer anything platform-shaped in it.
//
// It used to carry `using gpio_num_t = int;` for host builds. That shim existed
// for exactly one reason: ButtonConfig held a `pin` this layer never used for
// any decision. Splitting the wiring out (ButtonWiring, in button_manager.hpp)
// removed the field and the shim with it.
//
// See doc-design/button-esp-idf-design.md sections 5, 8, 9.
#pragma once

#include <cstdint>

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

// How the button should BEHAVE. Nothing here says anything about how it is
// wired -- that is ButtonWiring, and it belongs to the manager.
//
// Every field is a gesture parameter this layer either reads directly or (for
// debounceMs) converts into one through debounceCountFor().
struct ButtonBehavior {
    bool     enableDoubleClick{true};

    // Off by default, because turning it on inserts an extra event in front of
    // every gesture and would change the sequence an existing app receives.
    //
    // Turn it on when something must react at the instant of the press -- a
    // feedback beep, a haptic tick, a screen waking up. Those cannot wait for
    // CLICK: with enableDoubleClick on, CLICK is held back by doubleClickMs
    // while the FSM rules out a second press (§9.2), which is a delay a finger
    // can feel. PRESS_DOWN reports the pin, not a gesture, so nothing is pending
    // and it goes out as soon as the debounce integrator settles.
    bool     enablePressDown{false};

    uint32_t longPressMs{800};
    uint32_t doubleClickMs{250};

    // Real time. Button itself never reads this field: the CALLER turns it into
    // a cycle count with debounceCountFor() and passes that to the constructor,
    // because only the caller knows the poll interval. It stays here, next to
    // the other timings, so a single struct describes the whole gesture feel.
    uint32_t debounceMs{20};
};

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
    Button(const ButtonBehavior& behavior, uint8_t debounceMaxCnt);

    // Call once per poll cycle. rawPressed is already normalised for the wiring
    // (true means pressed). Returns ONE event, or NONE.
    // Precedence rule: edges first, timeouts second (§9.2).
    ButtonEvent update(bool rawPressed, uint32_t nowMs);

    bool        isPressed() const { return stableState_; }
    ButtonState getState()  const { return fsmState_; }

    const ButtonBehavior& behavior() const { return behavior_; }

    // The cycle count actually in force, after the zero clamp.
    uint8_t debounceMaxCnt() const { return debounceMaxCnt_; }

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

    ButtonBehavior behavior_;
    uint8_t        debounceMaxCnt_;

    uint8_t     debounceCounter_{0};
    bool        stableState_{false};
    bool        prevStable_{false};
    ButtonState fsmState_{ButtonState::IDLE};
    uint32_t    pressTimestamp_{0};
    uint32_t    clickTimestamp_{0};
};

}  // namespace button
