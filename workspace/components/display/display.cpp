#include "display.hpp"

#include <cmath>

namespace display {

// ------------------------------------------------------------------- geometry

bool isEmpty(const Area& a)
{
    return a.x1 <= a.x0 || a.y1 <= a.y0;
}

bool isValid(const Area& a, const Geometry& g)
{
    if (isEmpty(a)) {
        return false;
    }
    return a.x1 <= g.logicalWidth() && a.y1 <= g.logicalHeight();
}

Area fromInclusive(int32_t x1, int32_t y1, int32_t x2, int32_t y2)
{
    // Clamp before the +1 so a fully off-screen negative area collapses to
    // empty rather than wrapping into a huge unsigned rectangle.
    if (x2 < 0 || y2 < 0 || x2 < x1 || y2 < y1) {
        return Area{};
    }
    if (x1 < 0) {
        x1 = 0;
    }
    if (y1 < 0) {
        y1 = 0;
    }

    Area a;
    a.x0 = static_cast<uint16_t>(x1);
    a.y0 = static_cast<uint16_t>(y1);
    a.x1 = static_cast<uint16_t>(x2 + 1);
    a.y1 = static_cast<uint16_t>(y2 + 1);
    return a;
}

Area clipTo(const Area& a, const Geometry& g)
{
    const uint16_t w = g.logicalWidth();
    const uint16_t h = g.logicalHeight();

    if (isEmpty(a) || a.x0 >= w || a.y0 >= h) {
        return Area{};
    }

    Area out = a;
    if (out.x1 > w) {
        out.x1 = w;
    }
    if (out.y1 > h) {
        out.y1 = h;
    }
    return isEmpty(out) ? Area{} : out;
}

uint32_t pixelCount(const Area& a)
{
    if (isEmpty(a)) {
        return 0;
    }
    return static_cast<uint32_t>(a.x1 - a.x0) * static_cast<uint32_t>(a.y1 - a.y0);
}

uint32_t byteCountRgb565(const Area& a)
{
    return pixelCount(a) * 2u;
}

// ------------------------------------------------------------------ brightness

uint32_t dutyForPercent(uint8_t percent, uint8_t resolutionBits, bool gamma)
{
    if (resolutionBits == 0) {
        return 0;
    }
    if (resolutionBits > 20) {
        resolutionBits = 20;
    }
    if (percent > 100) {
        percent = 100;
    }

    // LEDC accepts a duty of exactly 2^bits, which is the fully-on case.
    const uint32_t full = 1u << resolutionBits;

    if (percent == 0) {
        return 0;
    }
    if (percent == 100) {
        return full;
    }

    if (!gamma) {
        return (full * percent) / 100u;
    }

    const double ratio = std::pow(static_cast<double>(percent) / 100.0, 2.2);
    const double duty  = ratio * static_cast<double>(full);

    uint32_t rounded = static_cast<uint32_t>(duty + 0.5);

    // A non-zero request must produce a non-zero duty. At 10 bits the gamma
    // curve drops 1% to 0.4, which would round to 0 and read as "the backlight
    // is broken" rather than "very dim".
    if (rounded == 0) {
        rounded = 1;
    }
    return rounded > full ? full : rounded;
}

// ------------------------------------------------------------- power sequencer

void PowerPolicy::onInitialized(uint8_t brightnessPercent)
{
    initialized_ = true;

    // EVERY FLAG HERE DESCRIBES THE HARDWARE, NOT THE INTENTION.
    //
    // The init sequence ends with the panel on (disp_on_off(true)) but with the
    // backlight channel still at duty 0 -- LEDC is configured first, precisely
    // so GPIO15 is held low from the start. So backlightUp_ is FALSE.
    //
    // An earlier version set it to true "because init() means the display is
    // up". The policy then believed the light was already at defaultBrightness,
    // never emitted APPLY_BRIGHTNESS, and the screen stayed dark until some
    // later setBrightness() happened to change the value. On the bring-up
    // firmware that was the brightness sweep seven seconds in, which is what
    // made the panel appear to blink before finally showing anything.
    //
    // The lesson generalises: this class is only correct while its flags are a
    // faithful model of the hardware. A flag set to what we WANT rather than to
    // what IS turns every derived step into a lie.
    backlightUp_ = false;
    panelOn_     = true;
    panelAwake_  = true;

    // GRAM contents are undefined after power-on, exactly as they are after
    // SLPIN/SLPOUT, so the same rule applies: somebody must paint before the
    // light comes up.
    redrawPending_ = true;

    quiesced_        = false;
    brightnessDirty_ = false;
    target_          = State::AWAKE;
    brightness_      = brightnessPercent > 100 ? 100 : brightnessPercent;
}

void PowerPolicy::request(Request r, uint8_t brightnessPercent)
{
    if (!initialized_) {
        return;
    }

    switch (r) {
    case Request::WAKE:
        target_ = State::AWAKE;
        break;

    case Request::SLEEP:
        target_ = State::PANEL_SLEEP;
        break;

    case Request::DIM:
        // DIM from PANEL_SLEEP is a no-op: the screen is already darker than
        // dimmed, and waking the panel just to dim it would cost 100 ms for
        // nothing.
        if (target_ != State::PANEL_SLEEP) {
            target_ = State::DIMMED;
        }
        break;

    case Request::SET_BRIGHTNESS: {
        const uint8_t clamped = brightnessPercent > 100 ? 100 : brightnessPercent;
        if (clamped != brightness_) {
            brightness_ = clamped;
            // Only needs re-applying if the backlight is currently lit;
            // otherwise the next APPLY_BRIGHTNESS picks up the new value.
            brightnessDirty_ = backlightUp_;
        }
        break;
    }
    }

    // quiesced_ means "no transfer can be in flight", so it stays true exactly
    // as long as flushes remain barred. The moment the target moves off
    // PANEL_SLEEP, acceptsFlush() may open again and any earlier wait is void.
    //
    // Clearing it on EVERY request instead would be wrong in a quiet way: a DIM
    // or a SET_BRIGHTNESS aimed at an already sleeping panel would re-arm a
    // WAIT_TRANSFER that has nothing to wait for, leaving the policy busy()
    // forever from the UI task's point of view.
    if (target_ != State::PANEL_SLEEP) {
        quiesced_ = false;
    }
}

Step PowerPolicy::nextStep() const
{
    if (!initialized_) {
        return Step::NONE;
    }

    switch (target_) {
    case State::AWAKE:
        if (!panelAwake_) {
            return Step::PANEL_SLEEP_OUT;
        }
        if (!panelOn_) {
            return Step::PANEL_ON;
        }
        // Rule 4 of section 11.2: GRAM contents are undefined across
        // SLPIN/SLPOUT, so the redraw has to land before the light comes up.
        // Lighting first shows the user a frame of garbage.
        if (redrawPending_) {
            return Step::REQUEST_REDRAW;
        }
        if (!backlightUp_ || brightnessDirty_) {
            return Step::APPLY_BRIGHTNESS;
        }
        return Step::NONE;

    case State::DIMMED:
        if (!panelAwake_) {
            return Step::PANEL_SLEEP_OUT;
        }
        if (!panelOn_) {
            return Step::PANEL_ON;
        }
        if (redrawPending_) {
            return Step::REQUEST_REDRAW;
        }
        if (backlightUp_) {
            return Step::BACKLIGHT_OFF;
        }
        return Step::NONE;

    case State::PANEL_SLEEP:
        if (backlightUp_) {
            return Step::BACKLIGHT_OFF;
        }
        // Only after the light is out, so a torn final frame is never visible.
        if (!quiesced_) {
            return Step::WAIT_TRANSFER;
        }
        if (panelOn_) {
            return Step::PANEL_OFF;
        }
        if (panelAwake_) {
            return Step::PANEL_SLEEP_IN;
        }
        return Step::NONE;

    case State::UNINITIALIZED:
        break;
    }
    return Step::NONE;
}

void PowerPolicy::stepDone()
{
    switch (nextStep()) {
    case Step::BACKLIGHT_OFF:
        backlightUp_     = false;
        brightnessDirty_ = false;
        break;

    case Step::APPLY_BRIGHTNESS:
        backlightUp_     = true;
        brightnessDirty_ = false;
        break;

    case Step::WAIT_TRANSFER:
        // Set even when the wait timed out. See the comment on stepDone().
        quiesced_ = true;
        break;

    case Step::PANEL_OFF:
        panelOn_ = false;
        break;

    case Step::PANEL_ON:
        panelOn_ = true;
        break;

    case Step::PANEL_SLEEP_IN:
        panelAwake_    = false;
        redrawPending_ = true;
        break;

    case Step::PANEL_SLEEP_OUT:
        panelAwake_ = true;
        break;

    case Step::REQUEST_REDRAW:
        redrawPending_ = false;
        break;

    case Step::NONE:
        break;
    }
}

bool PowerPolicy::acceptsFlush() const
{
    if (!initialized_) {
        return false;
    }
    return panelAwake_ && panelOn_ && target_ != State::PANEL_SLEEP;
}

State PowerPolicy::state() const
{
    if (!initialized_) {
        return State::UNINITIALIZED;
    }
    if (!panelAwake_ || !panelOn_) {
        return State::PANEL_SLEEP;
    }
    return backlightUp_ ? State::AWAKE : State::DIMMED;
}

void PowerPolicy::reset()
{
    *this = PowerPolicy{};
}

}  // namespace display
