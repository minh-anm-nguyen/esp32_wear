// Raw chip coordinates -> the coordinates the UI uses. Pure arithmetic, no
// includes beyond <cstdint>, so all eight orientations are testable on a PC --
// the same arrangement as axis_remap.hpp for the IMU.
//
// WHY THIS DOES NOT INCLUDE display.hpp
//
// The transform must have exactly ONE owner, or the driver and LVGL each apply
// half of it and the result is right in landscape and wrong in portrait. The
// owner here is the APPLICATION: main.cpp already knows display::Geometry, and
// building a TouchOrientation from it there is one line in the one place that
// legitimately sees both. Depending on `display` from `touch` would drag
// esp_lcd, SPI and LEDC into a component that only needs three booleans, and
// i2c-bus-design.md section 12 is explicit that there are no arrows between
// the device components.
//
// WHY THE LCD GAP IS NOT HERE
//
// display::Geometry carries yGap = 20, because the ST7789 holds 240x320 of GRAM
// and lights only 240x280 of it. That offset is a property of the DISPLAY
// CONTROLLER memory, not of the touch panel: the digitiser reports 0..279 over
// the glass regardless of where the pixels live in someone else's RAM. Adding
// it here would shift every touch by 20 pixels, and it would look almost right,
// which is the worst kind of wrong.
//
// Add it ONLY if a hardware test shows the chip really does report an offset,
// and then add it as its own field with the measurement written next to it.
#pragma once

#include <cstdint>

namespace touch {

// How the digitiser is oriented relative to what the user sees.
//
// EVERY FIELD DEFAULTS TO THE IDENTITY, on purpose. A wrong guess baked in as a
// default produces a panel that responds in the mirror image of where fingers
// land, and it is not obvious from a log which of the three flags is wrong. The
// identity is obviously wrong instead of subtly wrong, and TouchManager logs
// raw and logical coordinates side by side during bring-up so one corner test
// settles all three flags at once.
//
// The mapping from the display side, once measured, is a single line in
// main.cpp -- see the note at the top of this file.
struct TouchOrientation {
    // The panel as the CHIP numbers it, before any swap.
    uint16_t rawWidth{240};
    uint16_t rawHeight{280};

    // Applied in this order: swap, then mirror. The order matters and matches
    // what an LCD controller does with MADCTL, so a display setting can be
    // carried across without re-deriving it.
    bool swapXy{false};
    bool mirrorX{false};
    bool mirrorY{false};

    // Size of the coordinate space handed to the UI. Derived, not configured:
    // after a swap the width is the raw height.
    constexpr uint16_t logicalWidth() const { return swapXy ? rawHeight : rawWidth; }
    constexpr uint16_t logicalHeight() const { return swapXy ? rawWidth : rawHeight; }
};

// A point in UI coordinates.
struct TouchPoint {
    int16_t x{0};
    int16_t y{0};
    bool    valid{false};
};

// Raw -> logical. Returns valid == false when the input does not fit the raw
// space at all, rather than clamping: a coordinate that was already rejected by
// parseFrame() must not reappear here as a press at the edge.
//
// Note this takes the RAW coordinate that parseFrame() has already validated
// and edge-clamped. Those are two different jobs and stay in two places:
// parseFrame decides whether the chip said something sane, this decides where
// on the screen it means.
TouchPoint toLogical(uint16_t rawX, uint16_t rawY, const TouchOrientation& o);

}  // namespace touch
