// Pure logic layer: screen geometry, the brightness curve, and the power
// sequencer. No esp_lcd, no LEDC, no FreeRTOS, no LVGL.
//
// This header includes nothing but <cstdint> and <cmath>, so it compiles with a
// bare g++ and every rule below is testable on a PC. The ESP-IDF half lives in
// display_manager.hpp -- same split as button.hpp/button_manager.hpp and
// buzzer.hpp/buzzer_manager.hpp.
//
// Splitting here is not symmetry for its own sake: PowerPolicy is the part of
// this component most likely to be wrong, and on hardware it can only be
// checked by holding the watch and staring at it.
//
// See doc-design/display.md sections 6, 7.1, 9.3 and 11.
#pragma once

#include <cstdint>

namespace display {

// ------------------------------------------------------------------- geometry

// x1/y1 are EXCLUSIVE, matching esp_lcd_panel_draw_bitmap().
//
// LVGL's lv_area_t is INCLUSIVE at both ends. Feeding one straight into the
// other loses the last row and column of every flush -- a defect almost
// invisible on a 240x280 panel. Convert with fromInclusive(), never by hand.
struct Area {
    uint16_t x0{};
    uint16_t y0{};
    uint16_t x1{};
    uint16_t y1{};

    bool operator==(const Area& o) const
    {
        return x0 == o.x0 && y0 == o.y0 && x1 == o.x1 && y1 == o.y1;
    }
};

struct Geometry {
    uint16_t width{240};   // visible panel, in the orientation the user sees
    uint16_t height{280};
    uint16_t gramWidth{240};   // ST7789 carries 240x320 of RAM ...
    uint16_t gramHeight{320};  // ... but this panel only lights 240x280
    uint16_t xGap{0};
    uint16_t yGap{20};

    bool mirrorX{true};
    bool mirrorY{true};
    bool swapXy{false};

    // swap_xy exchanges the axes, so every bounds check must use these rather
    // than width/height directly.
    constexpr uint16_t logicalWidth() const { return swapXy ? height : width; }
    constexpr uint16_t logicalHeight() const { return swapXy ? width : height; }

    // THE invariant that lets mirror and gap be configured independently.
    //
    // esp_lcd applies the gap as a plain addition to the coordinates while
    // mirror writes MADCTL, which flips which end of GRAM the panel reads from.
    // Those two only stay consistent when the visible window sits centred in
    // GRAM -- here 20 + 280 + 20 == 320.
    //
    // On an off-centre panel, flipping mirrorY would slide the image by
    // 2*yGap pixels and nothing would report an error. Asserted at init()
    // rather than trusted to whoever edits Config next.
    constexpr bool isCentredInGram() const
    {
        return (2u * xGap + width == gramWidth)
            && (2u * yGap + height == gramHeight);
    }
};

bool isEmpty(const Area& a);

// Rejects empty, inverted (x1 < x0) and out-of-bounds areas in one call. The
// inverted case matters: Area is unsigned, so a negative width computed
// upstream arrives here as a huge number, not as a negative one.
bool isValid(const Area& a, const Geometry& g);

// LVGL (inclusive, signed) -> Area (exclusive, unsigned). Negative coordinates
// clamp to 0; LVGL can hand out areas that start off-screen.
Area fromInclusive(int32_t x1, int32_t y1, int32_t x2, int32_t y2);

// Intersection with the visible area. Returns an empty Area when disjoint.
Area clipTo(const Area& a, const Geometry& g);

// uint32_t, not uint16_t: a full frame is 240*280 = 67 200 pixels and
// 134 400 bytes, both past the 65 535 a uint16_t holds.
uint32_t pixelCount(const Area& a);
uint32_t byteCountRgb565(const Area& a);

// ------------------------------------------------------------------ brightness

// percent 0..100 -> LEDC duty 0..2^bits. The LEDC duty range is inclusive of
// 2^bits, so 100% maps to (1 << bits), not (1 << bits) - 1.
//
// gamma == true applies an exponent of 2.2 to match the eye's response. Linear
// duty looks wrong on a watch: 10% and 30% are nearly indistinguishable at the
// bottom of the range while the top half does almost nothing.
uint32_t dutyForPercent(uint8_t percent, uint8_t resolutionBits, bool gamma);

// ------------------------------------------------------------- power sequencer

enum class State : uint8_t {
    UNINITIALIZED,
    AWAKE,        // panel on, backlight at the configured brightness
    DIMMED,       // backlight off, panel STILL ON -- flushes are still accepted
    PANEL_SLEEP,  // DISPOFF + SLPIN, flushes rejected
};

enum class Request : uint8_t {
    WAKE,
    SLEEP,
    DIM,
    SET_BRIGHTNESS,
};

// One atomic action for the ESP-IDF layer to carry out. Same shape as
// buzzer::ToneOutput: the pure layer decides WHAT and IN WHAT ORDER, the driver
// layer only obeys.
enum class Step : uint8_t {
    NONE,  // sequence complete

    // These two were FADE_BACKLIGHT_DOWN / FADE_BACKLIGHT_UP until the step
    // trace printed "step FADE_BL_UP ok duty=0" while dimming to zero. The step
    // does not mean "get brighter", it means "put brightness_ on the pin" --
    // and a log line that contradicts itself costs more time than the name
    // saved. BACKLIGHT_OFF forces 0 regardless of brightness_.
    BACKLIGHT_OFF,
    APPLY_BRIGHTNESS,
    WAIT_TRANSFER,    // must carry a timeout, see display.md section 7.3
    PANEL_OFF,
    PANEL_ON,
    PANEL_SLEEP_IN,
    PANEL_SLEEP_OUT,
    REQUEST_REDRAW,   // UI task marks the whole screen dirty
};

// Drives sleep/wake as a sequence of Steps, one per stepDone().
//
// WHY THIS TRACKS FACTS RATHER THAN A QUEUE OF STEPS
//
// A first version queued the step list at request() time. That cannot honour
// "last state wins": a WAKE arriving three steps into a sleep sequence would
// have to cancel a half-executed list and guess what the hardware had already
// done. Here nextStep() is derived from what has ACTUALLY been applied
// (backlightUp_, panelOn_, panelAwake_) plus the target, so reversing direction
// mid-sequence needs no cancellation logic at all -- the next call simply
// derives a different step.
//
// That matters on a wrist: raising and lowering the arm twice in a second is
// ordinary, and a queue would stack up stale commands and blink the screen.
class PowerPolicy {
public:
    // Records the state the init sequence of display.md section 8 actually
    // leaves behind: panel ON, backlight still at duty 0, GRAM undefined. So the
    // policy starts in DIMMED aiming at AWAKE, and owes a redraw and a fade.
    void onInitialized(uint8_t brightnessPercent);

    // brightnessPercent is only read for SET_BRIGHTNESS; ignored otherwise.
    void request(Request r, uint8_t brightnessPercent = 0);

    // Pure. Returns NONE when there is nothing left to do.
    Step nextStep() const;

    // Applies the effect of whatever nextStep() currently returns.
    //
    // Deliberately takes no argument: passing the Step back in would let the
    // caller's idea of the current step drift from the policy's. Call it after
    // executing the step -- INCLUDING after a failed one. A WAIT_TRANSFER that
    // timed out must still advance, because leaving the screen lit is worse
    // than sleeping with a transfer possibly in flight (section 7.3).
    void stepDone();

    // DIMMED still accepts flushes: turning the backlight off is not the same
    // as stopping drawing, and re-lighting a dark-but-live panel is instant
    // while leaving PANEL_SLEEP costs over 100 ms (section 11.3).
    //
    // The target_ term is what closes the race in rule 2 of section 11.2: once
    // sleep is requested, new flushes must stop IMMEDIATELY, not once PANEL_OFF
    // has been executed. Otherwise LVGL can start a flush in the window between
    // WAIT_TRANSFER and PANEL_OFF -- exactly the case WAIT_TRANSFER exists to
    // rule out.
    bool acceptsFlush() const;

    State   state() const;
    State   target() const { return target_; }
    uint8_t brightness() const { return brightness_; }
    bool    busy() const { return nextStep() != Step::NONE; }

    void reset();

private:
    bool initialized_{false};

    // What has actually been applied to the hardware.
    bool backlightUp_{false};
    bool panelOn_{false};
    bool panelAwake_{false};
    bool redrawPending_{false};
    bool quiesced_{false};        // a WAIT_TRANSFER has run since the last request
    bool brightnessDirty_{false};

    State   target_{State::UNINITIALIZED};
    uint8_t brightness_{0};
};

}  // namespace display
