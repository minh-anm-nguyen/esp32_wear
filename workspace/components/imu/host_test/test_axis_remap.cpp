// Tests for AxisRemap and the calibration arithmetic, on a PC with g++.
//   ./run_tests.sh
//
// The interactive half (prompts, stillness detection) is not tested here -- it
// needs a person and a chip. What IS tested is everything that turns three
// measured vectors into a mapping, including every way that can go wrong: a
// watch held at an angle, a watch never actually turned, a watch being moved.
#include "axis_remap.hpp"

#include <cmath>
#include <cstdio>

using namespace imu;

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

static bool nearly(float a, float b, float tol = 0.001f)
{
    return std::fabs(a - b) <= tol;
}

// A reading from a chip whose sensor axes are a known signed permutation of the
// screen axes. Given the screen axis pointing at the sky, produce what such a
// chip would report.
static void simulate(const AxisRemap& truth, int screenAxisUp, float out[3])
{
    out[0] = out[1] = out[2] = 0.0f;
    // Screen axis `screenAxisUp` reads +1 g; it is fed by sensor axis
    // truth.map[screenAxisUp] through truth.sign[screenAxisUp].
    out[truth.map[screenAxisUp]] =
        static_cast<float>(truth.sign[screenAxisUp]) * 1.0f;
}

// ----------------------------------------------------------------- the tests

static void testIdentityRemapIsANoOp()
{
    AxisRemap  r{};
    const float in[3] = {0.1f, -0.2f, 0.97f};
    float       out[3]{};
    r.apply(in, out);

    EXPECT(nearly(out[0], 0.1f) && nearly(out[1], -0.2f) && nearly(out[2], 0.97f),
           "remap mac dinh phai la identity");
    EXPECT(r.isValid(), "identity phai hop le");
}

static void testRemapAppliesPermutationAndSign()
{
    AxisRemap r{};
    r.map[0] = 1; r.sign[0] = -1;   // screen +X <- sensor -Y
    r.map[1] = 2; r.sign[1] = 1;    // screen +Y <- sensor +Z
    r.map[2] = 0; r.sign[2] = -1;   // screen +Z <- sensor -X

    const float in[3] = {0.3f, 0.5f, 0.8f};
    float       out[3]{};
    r.apply(in, out);

    EXPECT(nearly(out[0], -0.5f), "X phai lay -Y, duoc %.3f", out[0]);
    EXPECT(nearly(out[1], 0.8f), "Y phai lay +Z, duoc %.3f", out[1]);
    EXPECT(nearly(out[2], -0.3f), "Z phai lay -X, duoc %.3f", out[2]);
}

static void testInvalidRemapsRejected()
{
    AxisRemap dup{};
    dup.map[0] = 1; dup.map[1] = 1; dup.map[2] = 2;   // sensor Y used twice
    EXPECT(!dup.isValid(), "dung trung mot truc cam bien phai bi tu choi");

    AxisRemap range{};
    range.map[0] = 3;
    EXPECT(!range.isValid(), "chi so truc ngoai 0..2 phai bi tu choi");

    AxisRemap badSign{};
    badSign.sign[1] = 0;
    EXPECT(!badSign.isValid(), "dau phai la +1 hoac -1");
}

static void testDeriveRecoversIdentity()
{
    AxisCalibrator cal;
    const AxisRemap truth{};  // chip glued straight

    float v[3];
    simulate(truth, 2, v); EXPECT(cal.setPose(AxisCalibrator::Pose::ScreenUp, v), "ScreenUp");
    simulate(truth, 1, v); EXPECT(cal.setPose(AxisCalibrator::Pose::TopUp, v), "TopUp");
    simulate(truth, 0, v); EXPECT(cal.setPose(AxisCalibrator::Pose::RightUp, v), "RightUp");

    EXPECT(cal.complete(), "du ba tu the");

    AxisRemap got{};
    EXPECT(cal.derive(got), "phai suy ra duoc");
    for (int i = 0; i < 3; ++i) {
        EXPECT(got.map[i] == truth.map[i] && got.sign[i] == truth.sign[i],
               "truc %d: duoc map=%d sign=%d, mong map=%d sign=%d", i, got.map[i],
               got.sign[i], truth.map[i], truth.sign[i]);
    }
}

static void testDeriveRecoversRotatedChip()
{
    // The realistic case: the chip is glued 90 degrees round and upside down.
    AxisRemap truth{};
    truth.map[0] = 1; truth.sign[0] = 1;
    truth.map[1] = 0; truth.sign[1] = -1;
    truth.map[2] = 2; truth.sign[2] = -1;

    AxisCalibrator cal;
    float v[3];
    simulate(truth, 2, v); cal.setPose(AxisCalibrator::Pose::ScreenUp, v);
    simulate(truth, 1, v); cal.setPose(AxisCalibrator::Pose::TopUp, v);
    simulate(truth, 0, v); cal.setPose(AxisCalibrator::Pose::RightUp, v);

    AxisRemap got{};
    EXPECT(cal.derive(got), "phai suy ra duoc");
    for (int i = 0; i < 3; ++i) {
        EXPECT(got.map[i] == truth.map[i] && got.sign[i] == truth.sign[i],
               "truc %d sai: map=%d sign=%d", i, got.map[i], got.sign[i]);
    }

    // And the derived remap must actually undo the chip's orientation: feed it
    // a sensor reading for "face up" and the screen frame must say (0,0,+1).
    simulate(truth, 2, v);
    float screen[3]{};
    got.apply(v, screen);
    EXPECT(nearly(screen[2], 1.0f), "sau remap, nam ngua phai cho Z = +1, duoc %.2f",
           screen[2]);
}

static void testDeriveHandlesEveryOrientation()
{
    // Exhaustive: all 6 permutations x 8 sign combinations. Only the 48 that
    // form a signed permutation are reachable, and every one must round-trip.
    const int perms[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
    int tested = 0;

    for (const auto& p : perms) {
        for (int bits = 0; bits < 8; ++bits) {
            AxisRemap truth{};
            for (int i = 0; i < 3; ++i) {
                truth.map[i]  = static_cast<int8_t>(p[i]);
                truth.sign[i] = (bits & (1 << i)) ? -1 : 1;
            }

            AxisCalibrator cal;
            float v[3];
            simulate(truth, 2, v); cal.setPose(AxisCalibrator::Pose::ScreenUp, v);
            simulate(truth, 1, v); cal.setPose(AxisCalibrator::Pose::TopUp, v);
            simulate(truth, 0, v); cal.setPose(AxisCalibrator::Pose::RightUp, v);

            AxisRemap got{};
            const bool ok = cal.derive(got);
            bool same = ok;
            for (int i = 0; i < 3 && same; ++i) {
                same = (got.map[i] == truth.map[i] && got.sign[i] == truth.sign[i]);
            }
            EXPECT(same, "huong lap dat (%d,%d,%d) dau %d khong khoi phuc duoc",
                   p[0], p[1], p[2], bits);
            ++tested;
        }
    }
    EXPECT(tested == 48, "phai thu du 48 huong lap dat, thu %d", tested);
}

static void testTiltedPoseRejected()
{
    // A watch held at 45 degrees has no single axis pointing up. Accepting it
    // would bake a wrong mapping into the firmware, and nothing later would
    // reveal it -- so it must be refused at the source.
    AxisCalibrator cal;
    const float tilted[3] = {0.0f, 0.707f, 0.707f};
    EXPECT(!cal.setPose(AxisCalibrator::Pose::ScreenUp, tilted),
           "cam nghieng 45 do phai bi tu choi");
    EXPECT(!cal.hasPose(AxisCalibrator::Pose::ScreenUp), "tu the sai khong duoc luu");

    // Slightly off is fine: a hand on a table is never perfect.
    const float slightly[3] = {0.05f, -0.1f, 0.99f};
    EXPECT(cal.setPose(AxisCalibrator::Pose::ScreenUp, slightly),
           "lech nhe phai duoc chap nhan");
}

static void testMovingWatchRejected()
{
    AxisCalibrator cal;
    const float shaken[3] = {0.0f, 0.0f, 2.5f};   // being accelerated
    EXPECT(!cal.setPose(AxisCalibrator::Pose::ScreenUp, shaken),
           "|a| = 2.5 g phai bi tu choi");

    const float falling[3] = {0.0f, 0.0f, 0.1f};  // free fall
    EXPECT(!cal.setPose(AxisCalibrator::Pose::ScreenUp, falling),
           "|a| = 0.1 g phai bi tu choi");
}

static void testDeriveFailsOnIncompleteOrDegenerate()
{
    AxisCalibrator partial;
    float          v[3] = {0.0f, 0.0f, 1.0f};
    partial.setPose(AxisCalibrator::Pose::ScreenUp, v);
    AxisRemap out{};
    EXPECT(!partial.complete(), "thieu tu the thi chua complete()");
    EXPECT(!partial.derive(out), "thieu tu the thi khong duoc suy ra");

    // The operator never actually turned the watch: two poses report the same
    // sensor axis. That is not a permutation and must be caught.
    AxisCalibrator degenerate;
    float same[3] = {0.0f, 0.0f, 1.0f};
    degenerate.setPose(AxisCalibrator::Pose::ScreenUp, same);
    degenerate.setPose(AxisCalibrator::Pose::TopUp, same);
    float other[3] = {1.0f, 0.0f, 0.0f};
    degenerate.setPose(AxisCalibrator::Pose::RightUp, other);
    EXPECT(degenerate.complete(), "du ba tu the (du sai)");
    EXPECT(!degenerate.derive(out),
           "hai tu the trung truc cam bien phai bi tu choi, khong duoc tra ve "
           "mot remap lam mat mot truc");
}

static void testResetClearsPoses()
{
    AxisCalibrator cal;
    float          v[3] = {0.0f, 0.0f, 1.0f};
    cal.setPose(AxisCalibrator::Pose::ScreenUp, v);
    EXPECT(cal.hasPose(AxisCalibrator::Pose::ScreenUp), "da luu tu the");

    cal.reset();
    EXPECT(!cal.hasPose(AxisCalibrator::Pose::ScreenUp), "reset() phai xoa");
    EXPECT(!cal.complete(), "reset() phai lam chua hoan tat");
}

int main()
{
    struct Test {
        const char* name;
        void (*fn)();
    };

    const Test tests[] = {
        {"remap mac dinh la identity",             testIdentityRemapIsANoOp},
        {"remap ap dung hoan vi va dau",           testRemapAppliesPermutationAndSign},
        {"remap khong hop le bi tu choi",          testInvalidRemapsRejected},
        {"suy ra identity tu ba tu the",           testDeriveRecoversIdentity},
        {"suy ra duoc chip lap xoay + lat nguoc",  testDeriveRecoversRotatedChip},
        {"vet can du 48 huong lap dat",            testDeriveHandlesEveryOrientation},
        {"cam nghieng bi tu choi",                 testTiltedPoseRejected},
        {"dong ho dang chuyen dong bi tu choi",    testMovingWatchRejected},
        {"thieu tu the / trung truc -> tu choi",   testDeriveFailsOnIncompleteOrDegenerate},
        {"reset()",                                testResetClearsPoses},
    };

    std::printf("=== test AxisRemap + AxisCalibrator (logic thuan) ===\n");
    for (auto& t : tests) {
        int before = g_failed;
        t.fn();
        std::printf("  [%s] %s\n", g_failed == before ? "OK  " : "FAIL", t.name);
    }
    std::printf("--- %d kiem tra, %d that bai ---\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
