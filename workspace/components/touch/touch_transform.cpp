#include "touch_transform.hpp"

namespace touch {

TouchPoint toLogical(uint16_t rawX, uint16_t rawY, const TouchOrientation& o)
{
    TouchPoint out{};

    if (o.rawWidth == 0 || o.rawHeight == 0) {
        return out;
    }
    if (rawX >= o.rawWidth || rawY >= o.rawHeight) {
        // Rejected, not clamped. See the header: a bad coordinate must not be
        // rescued into a plausible press.
        return out;
    }

    uint16_t x = rawX;
    uint16_t y = rawY;

    // Swap first, so the mirrors below operate on the axes the UI sees. Doing
    // it the other way round gives a different result for any orientation that
    // both swaps and mirrors -- which is every 90 and 270 degree rotation.
    if (o.swapXy) {
        const uint16_t t = x;
        x                = y;
        y                = t;
    }

    const uint16_t w = o.logicalWidth();
    const uint16_t h = o.logicalHeight();

    if (o.mirrorX) {
        x = static_cast<uint16_t>(w - 1 - x);
    }
    if (o.mirrorY) {
        y = static_cast<uint16_t>(h - 1 - y);
    }

    out.x     = static_cast<int16_t>(x);
    out.y     = static_cast<int16_t>(y);
    out.valid = true;
    return out;
}

}  // namespace touch
