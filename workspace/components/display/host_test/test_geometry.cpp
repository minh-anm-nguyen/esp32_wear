// Tests for the geometry and brightness helpers. Run on a PC with g++.
//   ./run_tests.sh
#include "display.hpp"

#include <cstdint>
#include <cstdio>

using namespace display;

// ------------------------------------------------------------ test harness

static int g_checks = 0;
static int g_failed = 0;

#define EXPECT(cond, ...)                                            \
    do {                                                             \
        ++g_checks;                                                  \
        if (!(cond)) {                                               \
            ++g_failed;                                              \
            std::printf("  FAIL  %s:%d  ", __FILE__, __LINE__);      \
            std::printf(__VA_ARGS__);                                \
            std::printf("\n");                                       \
        }                                                            \
    } while (0)

static Geometry board()
{
    return Geometry{};  // 240x280 in 240x320 of GRAM, gap 0/20
}

// ---------------------------------------------------------------- fromInclusive

static void test_from_inclusive()
{
    std::printf("fromInclusive\n");

    // The case that silently loses a row and a column if done by hand: LVGL
    // gives inclusive bounds, esp_lcd wants exclusive ones.
    const Area one = fromInclusive(10, 20, 10, 20);
    EXPECT(one.x0 == 10 && one.y0 == 20, "start kept");
    EXPECT(one.x1 == 11 && one.y1 == 21, "end is exclusive");
    EXPECT(pixelCount(one) == 1, "a 1x1 LVGL area is 1 pixel, got %u",
           pixelCount(one));

    const Area full = fromInclusive(0, 0, 239, 279);
    EXPECT(pixelCount(full) == 67200, "full screen pixels: %u", pixelCount(full));

    // LVGL can hand out areas that begin off-screen.
    const Area clamped = fromInclusive(-5, -5, 9, 9);
    EXPECT(clamped.x0 == 0 && clamped.y0 == 0, "negative start clamps to 0");
    EXPECT(clamped.x1 == 10 && clamped.y1 == 10, "end unaffected by clamping");

    // Fully negative, and inverted: both must collapse rather than wrap into a
    // huge unsigned rectangle.
    EXPECT(isEmpty(fromInclusive(-10, -10, -1, -1)), "fully off-screen is empty");
    EXPECT(isEmpty(fromInclusive(50, 50, 40, 40)), "inverted area is empty");
}

// ---------------------------------------------------------------------- isValid

static void test_is_valid()
{
    std::printf("isValid\n");

    const Geometry g = board();

    EXPECT(isValid(Area{0, 0, 240, 280}, g), "the whole screen is valid");
    EXPECT(isValid(Area{239, 279, 240, 280}, g), "the last pixel is valid");

    EXPECT(!isValid(Area{0, 0, 0, 0}, g), "empty rejected");
    EXPECT(!isValid(Area{10, 10, 10, 20}, g), "zero width rejected");
    EXPECT(!isValid(Area{10, 10, 20, 10}, g), "zero height rejected");

    // Area is unsigned, so an inverted rectangle arrives as x1 < x0 rather than
    // as a negative width. isEmpty() is what catches it.
    EXPECT(!isValid(Area{100, 10, 50, 20}, g), "x1 < x0 rejected");

    EXPECT(!isValid(Area{0, 0, 241, 280}, g), "one past the right edge rejected");
    EXPECT(!isValid(Area{0, 0, 240, 281}, g), "one past the bottom edge rejected");

    // The full GRAM height is NOT a valid target: 40 of those 320 rows are not
    // wired to the glass.
    EXPECT(!isValid(Area{0, 0, 240, 320}, g), "GRAM height is not panel height");
}

// ------------------------------------------------------------------- overflow

static void test_no_overflow()
{
    std::printf("byteCountRgb565\n");

    const Area full{0, 0, 240, 280};
    EXPECT(pixelCount(full) == 67200, "pixels: %u", pixelCount(full));

    // 134 400 is well past what a uint16_t holds. This is the test that
    // justifies the return type.
    EXPECT(byteCountRgb565(full) == 134400, "bytes: %u", byteCountRgb565(full));
    EXPECT(byteCountRgb565(full) > 65535, "a uint16_t return type would wrap");

    // The flush buffer of display.md section 9.1.
    const Area flush{0, 0, 240, 40};
    EXPECT(byteCountRgb565(flush) == 19200, "flush bytes: %u",
           byteCountRgb565(flush));

    EXPECT(pixelCount(Area{}) == 0, "empty area has no pixels");
    EXPECT(byteCountRgb565(Area{}) == 0, "empty area has no bytes");
}

// ----------------------------------------------------------------------- clipTo

static void test_clip()
{
    std::printf("clipTo\n");

    const Geometry g = board();

    EXPECT(clipTo(Area{0, 0, 240, 280}, g) == (Area{0, 0, 240, 280}),
           "an exact fit is unchanged");
    EXPECT(clipTo(Area{200, 250, 300, 400}, g) == (Area{200, 250, 240, 280}),
           "overhang is trimmed to the panel");
    EXPECT(isEmpty(clipTo(Area{240, 0, 260, 10}, g)), "starting at the edge is empty");
    EXPECT(isEmpty(clipTo(Area{300, 300, 320, 320}, g)), "fully outside is empty");
}

// ------------------------------------------------------------------- Geometry

static void test_geometry_invariants()
{
    std::printf("Geometry\n");

    const Geometry g = board();

    // 20 + 280 + 20 == 320. This is the only reason mirror and gap can be
    // configured independently -- see display.md section 9.3.
    EXPECT(g.isCentredInGram(), "the board panel is centred in GRAM");

    Geometry off = g;
    off.yGap = 30;  // 30 + 280 + 30 = 340 != 320
    EXPECT(!off.isCentredInGram(),
           "an off-centre panel must be rejected, not silently shifted");

    Geometry wrong = g;
    wrong.gramHeight = 240;
    EXPECT(!wrong.isCentredInGram(), "a panel taller than its GRAM is impossible");

    EXPECT(g.logicalWidth() == 240 && g.logicalHeight() == 280,
           "without swap_xy the axes are as configured");

    Geometry swapped = g;
    swapped.swapXy = true;
    EXPECT(swapped.logicalWidth() == 280 && swapped.logicalHeight() == 240,
           "swap_xy exchanges the axes used for bounds checks");
    EXPECT(isValid(Area{0, 0, 280, 240}, swapped),
           "a landscape area is valid once swapped");
    EXPECT(!isValid(Area{0, 0, 240, 280}, swapped),
           "the portrait area is out of bounds once swapped");
}

// ------------------------------------------------------------- dutyForPercent

static void test_duty()
{
    std::printf("dutyForPercent\n");

    const uint8_t  bits = 10;
    const uint32_t full = 1024;  // LEDC duty is inclusive of 2^bits

    EXPECT(dutyForPercent(0, bits, false) == 0, "0%% is off");
    EXPECT(dutyForPercent(0, bits, true) == 0, "0%% is off with gamma too");
    EXPECT(dutyForPercent(100, bits, false) == full, "100%% is fully on");
    EXPECT(dutyForPercent(100, bits, true) == full, "100%% is fully on with gamma");

    EXPECT(dutyForPercent(50, bits, false) == 512, "linear 50%% is half: %u",
           dutyForPercent(50, bits, false));

    // Gamma pulls the middle down -- that is the entire point of it.
    EXPECT(dutyForPercent(50, bits, true) < dutyForPercent(50, bits, false),
           "gamma darkens the midpoint");

    // Monotonic, or the fade would go backwards somewhere.
    uint32_t prev = 0;
    for (uint8_t p = 1; p <= 100; ++p) {
        const uint32_t d = dutyForPercent(p, bits, true);
        EXPECT(d >= prev, "gamma curve is monotonic at %u%%", p);
        prev = d;
    }

    // 1% under gamma 2.2 lands at 0.4 of a step. Rounding that to 0 would make
    // the lowest setting look like broken hardware.
    EXPECT(dutyForPercent(1, bits, true) > 0, "the dimmest non-zero step is not 0");

    EXPECT(dutyForPercent(200, bits, false) == full, "over 100%% clamps");
    EXPECT(dutyForPercent(50, 0, false) == 0, "0 bits of resolution yields 0");
}

int main()
{
    std::printf("== display geometry / brightness ==\n");

    test_from_inclusive();
    test_is_valid();
    test_no_overflow();
    test_clip();
    test_geometry_invariants();
    test_duty();

    std::printf("%d checks, %d failed\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
