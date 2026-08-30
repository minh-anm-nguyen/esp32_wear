#include "button.hpp"

namespace button {

Button::Button(const ButtonBehavior& behavior, uint8_t debounceMaxCnt)
    : behavior_(behavior),
      debounceMaxCnt_(debounceMaxCnt == 0 ? 1 : debounceMaxCnt)
{
}

void Button::reset()
{
    debounceCounter_ = 0;
    stableState_     = false;
    prevStable_      = false;
    fsmState_        = ButtonState::IDLE;
    pressTimestamp_  = 0;
    clickTimestamp_  = 0;
    holdStartMs_     = 0;
}

// Integrator with hysteresis: the stable state only flips when the counter
// REACHES A BOUNDARY. The middle band deliberately does nothing -- that is what
// stops the output from chattering.
void Button::updateDebounce(bool rawPressed)
{
    if (rawPressed) {
        // The upper clamp is mandatory: without it, holding the button for 3
        // seconds pushes the counter to 300, and releasing then takes 300 cycles
        // to register.
        if (debounceCounter_ < debounceMaxCnt_) {
            ++debounceCounter_;
        }
    } else {
        if (debounceCounter_ > 0) {
            --debounceCounter_;
        }
    }

    if (debounceCounter_ == debounceMaxCnt_) {
        stableState_ = true;
    } else if (debounceCounter_ == 0) {
        stableState_ = false;
    }
    // 0 < counter < max: leave stableState_ untouched
}

ButtonEvent Button::update(bool rawPressed, uint32_t nowMs)
{
    updateDebounce(rawPressed);

    // An edge is true for EXACTLY ONE cycle of a whole press. Reacting to the
    // level instead would fire 300 transitions while the button is held for 3s.
    const bool pressedEdge  = stableState_ && !prevStable_;
    const bool releasedEdge = !stableState_ && prevStable_;

    ButtonEvent ev = ButtonEvent::NONE;

    switch (fsmState_) {
    case ButtonState::IDLE:
        if (pressedEdge) {
            fsmState_       = ButtonState::PRESSED;
            pressTimestamp_ = nowMs;
            // The only event that reports the PIN rather than a gesture, which
            // is exactly why it needs no waiting: nothing is left to disambiguate.
            //
            // This branch is the sole source of PRESS_DOWN. The second press of
            // a double click arrives in WAIT_DOUBLE_CLICK, not here, and emits
            // DOUBLE_CLICK -- already immediate, and the one-event-per-cycle rule
            // means it could not emit both anyway.
            if (behavior_.enablePressDown) {
                ev = ButtonEvent::PRESS_DOWN;
            }
        }
        break;

    case ButtonState::PRESSED:
        // releasedEdge requires stableState_ == false while the long-press branch
        // requires stableState_ == true, so the two are structurally exclusive.
        if (releasedEdge) {
            if (behavior_.enableDoubleClick) {
                fsmState_       = ButtonState::WAIT_DOUBLE_CLICK;
                clickTimestamp_ = nowMs;
            } else {
                fsmState_ = ButtonState::IDLE;
                ev        = ButtonEvent::CLICK;
            }
        } else if (stableState_ &&
                   (nowMs - pressTimestamp_) >= behavior_.longPressMs) {
            fsmState_ = ButtonState::WAIT_RELEASE_LONG;
            ev        = ButtonEvent::LONG_PRESS;
        }
        break;

    case ButtonState::WAIT_DOUBLE_CLICK:
        // EDGE FIRST, TIMEOUT SECOND. When both hold in the same cycle
        // DOUBLE_CLICK wins -- the user did land the second press inside the
        // window.
        if (pressedEdge) {
            fsmState_    = ButtonState::WAIT_RELEASE;
            holdStartMs_ = nowMs;   // the hold is timed from HERE, not pressTimestamp_
            ev           = ButtonEvent::DOUBLE_CLICK;
        } else if ((nowMs - clickTimestamp_) >= behavior_.doubleClickMs) {
            fsmState_ = ButtonState::IDLE;
            ev        = ButtonEvent::CLICK;
        }
        break;

    case ButtonState::WAIT_RELEASE_LONG:
        if (releasedEdge) {
            fsmState_ = ButtonState::IDLE;
            ev        = ButtonEvent::LONG_PRESS_RELEASE;
        }
        break;

    case ButtonState::WAIT_RELEASE:
        // Swallow the tail of the press that DOUBLE_CLICK already consumed.
        // Without this exit the FSM latches forever and the manager task can
        // never go back to sleep.
        if (releasedEdge) {
            fsmState_ = ButtonState::IDLE;
        } else if (behavior_.enableDoubleClickHold && stableState_ &&
                   (nowMs - holdStartMs_) >= behavior_.longPressMs) {
            // Same structural exclusion as in PRESSED: releasedEdge needs
            // stableState_ == false, this branch needs it true.
            fsmState_ = ButtonState::WAIT_RELEASE_HOLD;
            ev        = ButtonEvent::DOUBLE_CLICK_HOLD;
        }
        break;

    case ButtonState::WAIT_RELEASE_HOLD:
        // DOUBLE_CLICK_HOLD has been reported; stay silent until the finger
        // lifts. Existing for the same reason as WAIT_RELEASE_LONG: without a
        // state to move into, the threshold would keep matching and the event
        // would fire on every poll cycle for as long as the button is held.
        if (releasedEdge) {
            fsmState_ = ButtonState::IDLE;
        }
        break;
    }

    prevStable_ = stableState_;
    return ev;
}

}  // namespace button
