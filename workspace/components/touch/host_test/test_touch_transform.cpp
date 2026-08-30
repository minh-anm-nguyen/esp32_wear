// Tests for the raw -> logical touch transform. Pure arithmetic, so all eight
// orientations are checked exhaustively on a PC.
//   ./run_tests.sh
//
// The bug this file exists to prevent: a transform that looks right in one
// orientation and is mirrored in another. Checking the CENTRE proves nothing --
// it is invariant under every mirror. Only the corners separate the eight
// cases, so every test here works on corners.
#include "touch_transform.hpp"

#include <cstdio>

using namespace touch;

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

static void expectPoint(const TouchOrientation& o, uint16_t rx, uint16_t ry,
                        int16_t wx, int16_t wy, const char* what)
{
    const TouchPoint p = toLogical(rx, ry, o);
    EXPECT(p.valid, "%s: (%u,%u) bi tu choi", what, rx, ry);
    EXPECT(p.x == wx && p.y == wy, "%s: (%u,%u) -> (%d,%d), mong doi (%d,%d)",
           what, rx, ry, p.x, p.y, wx, wy);
}

static void testIdentity()
{
    TouchOrientation o{};  // every flag false by default, on purpose
    EXPECT(o.logicalWidth() == 240 && o.logicalHeight() == 280, "kich thuoc logic");

    expectPoint(o, 0, 0, 0, 0, "identity goc tren-trai");
    expectPoint(o, 239, 279, 239, 279, "identity goc duoi-phai");
    expectPoint(o, 100, 200, 100, 200, "identity giua");
}

static void testMirrors()
{
    TouchOrientation o{};

    o         = TouchOrientation{};
    o.mirrorX = true;
    expectPoint(o, 0, 0, 239, 0, "mirrorX goc tren-trai");
    expectPoint(o, 239, 279, 0, 279, "mirrorX goc duoi-phai");

    o         = TouchOrientation{};
    o.mirrorY = true;
    expectPoint(o, 0, 0, 0, 279, "mirrorY goc tren-trai");
    expectPoint(o, 239, 279, 239, 0, "mirrorY goc duoi-phai");

    o         = TouchOrientation{};
    o.mirrorX = true;
    o.mirrorY = true;
    expectPoint(o, 0, 0, 239, 279, "mirror ca hai");
    expectPoint(o, 239, 279, 0, 0, "mirror ca hai, goc doi dien");
}

// swap_xy exchanges the axes, so the logical size changes with it. A transform
// that keeps using the raw width here is off by 40 pixels on this panel and
// still looks plausible.
static void testSwapChangesLogicalSize()
{
    TouchOrientation o{};
    o.swapXy = true;

    EXPECT(o.logicalWidth() == 280, "swap: logicalWidth=%u", o.logicalWidth());
    EXPECT(o.logicalHeight() == 240, "swap: logicalHeight=%u", o.logicalHeight());

    expectPoint(o, 0, 0, 0, 0, "swap goc tren-trai");
    expectPoint(o, 239, 279, 279, 239, "swap goc duoi-phai");
    expectPoint(o, 10, 250, 250, 10, "swap diem bat ky");
}

// The order swap-then-mirror is the whole content of a 90 degree rotation.
// Doing it the other way round gives a different answer, and this is the test
// that says which one this code means.
static void testSwapThenMirrorOrder()
{
    TouchOrientation o{};
    o.swapXy  = true;
    o.mirrorX = true;

    // swap: (0,0) -> (0,0); mirror X across the LOGICAL width 280 -> (279,0)
    expectPoint(o, 0, 0, 279, 0, "swap+mirrorX");
    expectPoint(o, 239, 279, 0, 239, "swap+mirrorX goc doi dien");

    o         = TouchOrientation{};
    o.swapXy  = true;
    o.mirrorY = true;
    // swap: (0,0) -> (0,0); mirror Y across the LOGICAL height 240 -> (0,239)
    expectPoint(o, 0, 0, 0, 239, "swap+mirrorY");
}

// Every one of the eight combinations must map the four corners onto the four
// corners -- bijectively. A transform that sends two different corners to the
// same place is broken in a way no single-point test can see.
static void testAllEightOrientationsAreBijectionsOnCorners()
{
    for (int mask = 0; mask < 8; ++mask) {
        TouchOrientation o{};
        o.swapXy  = (mask & 1) != 0;
        o.mirrorX = (mask & 2) != 0;
        o.mirrorY = (mask & 4) != 0;

        const uint16_t w = o.logicalWidth();
        const uint16_t h = o.logicalHeight();

        const uint16_t rawCorners[4][2] = {{0, 0}, {239, 0}, {0, 279}, {239, 279}};
        int            seen[4]          = {0, 0, 0, 0};

        for (auto& c : rawCorners) {
            const TouchPoint p = toLogical(c[0], c[1], o);
            EXPECT(p.valid, "mask=%d: goc (%u,%u) bi tu choi", mask, c[0], c[1]);

            const bool left = (p.x == 0);
            const bool top  = (p.y == 0);
            EXPECT(p.x == 0 || p.x == w - 1, "mask=%d: x=%d khong phai bien (w=%u)",
                   mask, p.x, w);
            EXPECT(p.y == 0 || p.y == h - 1, "mask=%d: y=%d khong phai bien (h=%u)",
                   mask, p.y, h);

            const int slot = (left ? 0 : 1) + (top ? 0 : 2);
            ++seen[slot];
        }
        for (int i = 0; i < 4; ++i) {
            EXPECT(seen[i] == 1, "mask=%d: goc thu %d duoc dung %d lan (phai la 1)",
                   mask, i, seen[i]);
        }
    }
}

// Out of range is REJECTED, never clamped. Clamping here would undo the whole
// point of parseFrame rejecting a corrupt coordinate: the bad value would come
// back as a perfectly valid press on the edge of the UI.
static void testRejectsRatherThanClamps()
{
    TouchOrientation o{};

    TouchPoint p = toLogical(240, 100, o);
    EXPECT(!p.valid, "x=240 phai bi tu choi, duoc (%d,%d)", p.x, p.y);

    p = toLogical(100, 280, o);
    EXPECT(!p.valid, "y=280 phai bi tu choi");

    p = toLogical(4095, 4095, o);
    EXPECT(!p.valid, "0xFFF phai bi tu choi");

    // A degenerate orientation must not divide the world by zero or wrap.
    TouchOrientation zero{};
    zero.rawWidth = 0;
    p             = toLogical(0, 0, zero);
    EXPECT(!p.valid, "rawWidth=0 phai bi tu choi");
}

int main()
{
    std::printf("touch_transform\n");

    testIdentity();
    testMirrors();
    testSwapChangesLogicalSize();
    testSwapThenMirrorOrder();
    testAllEightOrientationsAreBijectionsOnCorners();
    testRejectsRatherThanClamps();

    std::printf("  %d kiem tra, %d that bai\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
