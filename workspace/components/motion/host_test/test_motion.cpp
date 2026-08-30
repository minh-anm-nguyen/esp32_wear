// Tests for the wrist-raise FSM, run on a PC with g++. No chip, no ESP-IDF.
//   ./run_tests.sh
//
// Poses are described the way a person would: where the forearm is pointing and
// where the glass is facing. Everything that makes this class worth having --
// the two-angle decision, hysteresis, the settling window, the gravity gate,
// the re-arm rule -- is a property of a SEQUENCE, so that is what gets asserted.
#include "motion_controller.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace motion;

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

static bool nearly(float a, float b, float tol = 1.0f)
{
    return std::fabs(a - b) <= tol;
}

// Builds a sample from the two angles the FSM actually reasons about.
//   armPitch     -90 = hand straight down, 0 = forearm level, +90 = hand up
//   screenPitch  +90 = glass at the sky, 0 = glass vertical, -90 = glass down
// Whatever is left over goes on X, which neither angle uses.
static sensors::Sample pose(float armPitchDeg, float screenPitchDeg,
                            float magnitudeG = 1.0f)
{
    const float k = 0.017453292519943295f;
    const float y = std::sin(armPitchDeg * k);
    const float z = std::sin(screenPitchDeg * k);
    float       rest = 1.0f - y * y - z * z;
    if (rest < 0.0f) rest = 0.0f;

    sensors::Sample s{};
    s.accelG[0] = std::sqrt(rest) * magnitudeG;
    s.accelG[1] = y * magnitudeG;
    s.accelG[2] = z * magnitudeG;
    return s;
}

// The four poses a person actually holds a watch in, plus the resting one.
static sensors::Sample armHanging()      { return pose(-90.0f, 0.0f); }   // by the side
static sensors::Sample forearmLevelGlassUp() { return pose(0.0f, 85.0f); }
static sensors::Sample forearmAt45()     { return pose(45.0f, 40.0f); }
static sensors::Sample forearmUpGlassToEyes() { return pose(70.0f, 5.0f); }

struct Sim {
    MotionController         mc;
    uint32_t                 t{0};
    std::vector<MotionEvent> events;

    static constexpr uint32_t kStepMs = 16;  // 62.5 Hz, the real ACTIVITY rate

    explicit Sim(uint32_t startMs = 0) : t(startMs) {}

    void hold(const sensors::Sample& s, uint32_t durationMs)
    {
        for (uint32_t e = 0; e < durationMs; e += kStepMs) {
            const MotionEvent ev = mc.update(s, t);
            if (ev != MotionEvent::NONE) events.push_back(ev);
            t += kStepMs;
        }
    }

    void hold(float armPitch, float screenPitch, uint32_t durationMs,
              float mag = 1.0f)
    {
        for (uint32_t e = 0; e < durationMs; e += kStepMs) {
            const MotionEvent ev = mc.update(pose(armPitch, screenPitch, mag), t);
            if (ev != MotionEvent::NONE) events.push_back(ev);
            t += kStepMs;
        }
    }

    // Move smoothly between two poses.
    void sweep(float armFrom, float screenFrom, float armTo, float screenTo,
               uint32_t durationMs, float mag = 1.0f)
    {
        const uint32_t steps = (durationMs / kStepMs) ? (durationMs / kStepMs) : 1;
        for (uint32_t i = 0; i < steps; ++i) {
            const float f = static_cast<float>(i) / static_cast<float>(steps);
            const MotionEvent ev = mc.update(
                pose(armFrom + (armTo - armFrom) * f,
                     screenFrom + (screenTo - screenFrom) * f, mag), t);
            if (ev != MotionEvent::NONE) events.push_back(ev);
            t += kStepMs;
        }
    }

    size_t count(MotionEvent want) const
    {
        size_t n = 0;
        for (auto e : events) if (e == want) ++n;
        return n;
    }
};

static std::string describe(const std::vector<MotionEvent>& v)
{
    if (v.empty()) return "(khong co su kien)";
    std::string s;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) s += " , ";
        s += (v[i] == MotionEvent::WRIST_RAISE) ? "RAISE" : "LOWER";
    }
    return s;
}

// ----------------------------------------------------------------- the tests

static void testAngleGeometry()
{
    const float handDown[3] = {0.0f, -1.0f, 0.0f};
    const float level[3]    = {0.0f, 0.0f, 1.0f};
    const float handUp[3]   = {0.0f, 1.0f, 0.0f};

    EXPECT(nearly(armPitchDegFor(handDown), -90.0f), "tay buong = -90 do, duoc %.1f",
           armPitchDegFor(handDown));
    EXPECT(nearly(armPitchDegFor(level), 0.0f), "cang tay ngang = 0 do");
    EXPECT(nearly(armPitchDegFor(handUp), 90.0f), "tay gio len = +90 do");

    EXPECT(nearly(screenPitchDegFor(level), 90.0f), "kinh ngua len troi = +90 do");
    const float glassDown[3] = {0.0f, 0.0f, -1.0f};
    EXPECT(nearly(screenPitchDegFor(glassDown), -90.0f), "kinh up xuong = -90 do");

    // asin() outside [-1,1] is NaN, and NaN makes every later comparison false
    // without any error appearing anywhere.
    const float overUnit[3] = {0.0f, 1.0000001f, 0.0f};
    EXPECT(!std::isnan(armPitchDegFor(overUnit)), "lam tron KHONG duoc sinh NaN");
}

// THE regression this whole redesign exists for. All three reading poses must
// wake the screen; a single-angle model got two of them wrong.
static void testAllThreeReadingPosesWake()
{
    struct Case {
        const char*     name;
        sensors::Sample s;
    } cases[] = {
        {"cang tay ngang, kinh ngua len", forearmLevelGlassUp()},
        {"cang tay cheo 45 do",           forearmAt45()},
        {"tay gio cao, kinh dung truoc mat", forearmUpGlassToEyes()},
    };

    for (const auto& c : cases) {
        Sim s;
        s.hold(armHanging(), 400);   // start with the arm by the side
        s.hold(c.s, 600);            // then read the watch

        EXPECT(s.count(MotionEvent::WRIST_RAISE) == 1,
               "tu the '%s' phai bat man hinh, duoc %s", c.name,
               describe(s.events).c_str());
        EXPECT(s.mc.state() == MotionState::VIEWING, "'%s' phai o VIEWING", c.name);
    }
}

static void testArmHangingKeepsScreenOff()
{
    // The pose that must never wake it: forearm straight down. Note the screen
    // normal here is horizontal -- the SAME value as "arm up, glass to eyes",
    // which is exactly why one angle could not tell them apart.
    Sim s;
    s.hold(armHanging(), 2000);

    EXPECT(s.count(MotionEvent::WRIST_RAISE) == 0,
           "tay buong doc KHONG duoc bat man hinh, duoc %s", describe(s.events).c_str());
    EXPECT(s.mc.state() == MotionState::ARM_DOWN, "phai o ARM_DOWN");

    // And prove the ambiguity is real: both poses have a horizontal screen.
    EXPECT(nearly(screenPitchDegFor(armHanging().accelG), 0.0f, 6.0f),
           "tay buong: phap tuyen man hinh nam ngang");
    EXPECT(nearly(screenPitchDegFor(forearmUpGlassToEyes().accelG), 0.0f, 6.0f),
           "tay gio cao: phap tuyen man hinh CUNG nam ngang");
}

static void testArmUpButGlassFacingAwayDoesNotWake()
{
    // Forearm is up, so the arm test passes -- but the glass is turned towards
    // the floor, so there is nothing to read. The screen veto is what catches
    // this, and without it the forearm angle alone would wake it.
    Sim s;
    s.hold(armHanging(), 400);
    s.hold(30.0f, -70.0f, 800);

    EXPECT(s.count(MotionEvent::WRIST_RAISE) == 0,
           "tay gio len nhung kinh up xuong dat -> khong bat, duoc %s",
           describe(s.events).c_str());
}

static void testLoweringTurnsScreenOff()
{
    Sim s;
    s.hold(armHanging(), 400);
    s.hold(forearmLevelGlassUp(), 600);
    EXPECT(s.count(MotionEvent::WRIST_RAISE) == 1, "bat man hinh");

    s.sweep(0.0f, 85.0f, -90.0f, 0.0f, 500);   // arm back down
    s.hold(armHanging(), 400);

    EXPECT(s.count(MotionEvent::WRIST_LOWER) == 1, "ha tay phai tat man hinh");
    EXPECT(s.mc.state() == MotionState::ARM_DOWN, "phai ve ARM_DOWN");
}

static void testRaiseTooSlowIsIgnored()
{
    // An arm drifting up over several seconds is a change of posture, not
    // someone checking the time. Without the window this wakes in a pocket.
    Sim s;
    s.hold(armHanging(), 400);
    s.sweep(-90.0f, 0.0f, 0.0f, 85.0f, 4000);
    s.hold(forearmLevelGlassUp(), 600);

    EXPECT(s.count(MotionEvent::WRIST_RAISE) == 0,
           "nhac qua cham KHONG duoc bat man hinh, duoc %s", describe(s.events).c_str());
}

static void testSweepingThroughDoesNotWake()
{
    Sim s;
    s.hold(armHanging(), 400);
    s.sweep(-90.0f, 0.0f, 20.0f, 85.0f, 150);   // fast lift...
    s.sweep(20.0f, 85.0f, -90.0f, 0.0f, 150);   // ...and straight back down

    EXPECT(s.count(MotionEvent::WRIST_RAISE) == 0,
           "quet qua tu the xem ma khong dung lai -> khong bat, duoc %s",
           describe(s.events).c_str());
}

static void testShakingIsFilteredByGravityGate()
{
    Sim s;
    s.hold(armHanging(), 400);
    s.hold(0.0f, 85.0f, 800, 2.5f);   // right pose, but being flung at 2.5 g

    EXPECT(s.count(MotionEvent::WRIST_RAISE) == 0,
           "|a| = 2.5 g phai bi loc, duoc %s", describe(s.events).c_str());
    EXPECT(s.mc.inMotion(), "phai bao la dang bi gia toc");

    Sim f;
    f.hold(armHanging(), 400);
    f.hold(0.0f, 85.0f, 800, 0.1f);   // free fall
    EXPECT(f.count(MotionEvent::WRIST_RAISE) == 0, "|a| = 0.1 g cung phai bi loc");
}

static void testNoSecondRaiseWithoutLowering()
{
    // Small adjustments while reading must not re-fire.
    Sim s;
    s.hold(armHanging(), 400);
    s.hold(forearmLevelGlassUp(), 500);

    s.hold(forearmAt45(), 400);
    s.hold(forearmUpGlassToEyes(), 400);
    s.hold(forearmLevelGlassUp(), 400);

    EXPECT(s.count(MotionEvent::WRIST_RAISE) == 1,
           "doi tu the trong khi xem KHONG duoc bat lai, duoc %s",
           describe(s.events).c_str());
}

static void testHysteresisNoFlickerAtBoundary()
{
    Sim s;
    s.hold(armHanging(), 400);
    s.hold(forearmLevelGlassUp(), 500);
    const size_t afterRaise = s.events.size();

    // Hover the forearm right on the outer threshold for two seconds.
    for (int i = 0; i < 60; ++i) {
        s.hold(-44.0f, 60.0f, 16);
        s.hold(-46.0f, 60.0f, 16);
    }

    EXPECT(s.events.size() - afterRaise <= 1,
           "dao quanh nguong KHONG duoc sinh chuoi su kien, them %u",
           static_cast<unsigned>(s.events.size() - afterRaise));
}

static void testUint32Wrap()
{
    Sim s(0xFFFFF000u);
    s.hold(armHanging(), 400);
    s.hold(forearmLevelGlassUp(), 600);

    EXPECT(s.count(MotionEvent::WRIST_RAISE) == 1,
           "vat qua diem tran uint32 van phai dung, duoc %s",
           describe(s.events).c_str());
}

static void testCallbackPathForwardsEvents()
{
    struct Sink { int raises{0}; int lowers{0}; uint32_t lastMs{0}; } sink;

    MotionController mc;
    mc.setEventCallback(
        [](void* ctx, MotionEvent ev, uint32_t nowMs) {
            auto* s = static_cast<Sink*>(ctx);
            if (ev == MotionEvent::WRIST_RAISE) ++s->raises;
            if (ev == MotionEvent::WRIST_LOWER) ++s->lowers;
            s->lastMs = nowMs;
        },
        &sink);

    uint32_t t = 0;
    auto feed = [&](const sensors::Sample& s, uint32_t ms) {
        for (uint32_t e = 0; e < ms; e += 16) {
            mc.onSample(s, t);   // through ISampleSink, not update()
            t += 16;
        }
    };

    feed(armHanging(), 400);
    feed(forearmLevelGlassUp(), 600);
    EXPECT(sink.raises == 1, "callback phai nhan dung mot RAISE, duoc %d", sink.raises);
    EXPECT(sink.lastMs != 0, "callback phai nhan duoc moc thoi gian");

    feed(armHanging(), 600);
    EXPECT(sink.lowers == 1, "callback phai nhan mot LOWER, duoc %d", sink.lowers);
}

static void testResetReturnsToInitialState()
{
    Sim s;
    s.hold(armHanging(), 400);
    s.hold(forearmLevelGlassUp(), 500);
    EXPECT(s.mc.state() == MotionState::VIEWING, "dang o VIEWING truoc khi reset");

    s.mc.reset();
    EXPECT(s.mc.state() == MotionState::ARM_DOWN, "reset() phai ve ARM_DOWN");
    EXPECT(!s.mc.inMotion(), "reset() phai xoa co dang chuyen dong");
}

int main()
{
    struct Test {
        const char* name;
        void (*fn)();
    };

    const Test tests[] = {
        {"hinh hoc hai goc (cang tay / man hinh)",     testAngleGeometry},
        {"CA BA tu the xem gio deu bat man hinh",      testAllThreeReadingPosesWake},
        {"tay buong doc -> man hinh tat",              testArmHangingKeepsScreenOff},
        {"tay gio len nhung kinh up xuong -> khong bat", testArmUpButGlassFacingAwayDoesNotWake},
        {"ha tay -> tat man hinh",                     testLoweringTurnsScreenOff},
        {"nhac qua cham -> khong su kien",             testRaiseTooSlowIsIgnored},
        {"quet qua tu the xem, khong dung -> khong bat", testSweepingThroughDoesNotWake},
        {"cong loc trong luc: lac 2.5g va roi tu do",  testShakingIsFilteredByGravityGate},
        {"doi tu the khi dang xem -> khong bat lai",   testNoSecondRaiseWithoutLowering},
        {"hysteresis: khong nhap nhay o nguong",       testHysteresisNoFlickerAtBoundary},
        {"nowMs vat qua diem tran uint32",             testUint32Wrap},
        {"duong callback qua ISampleSink",             testCallbackPathForwardsEvents},
        {"reset()",                                    testResetReturnsToInitialState},
    };

    std::printf("=== test MotionController (logic thuan) ===\n");
    for (auto& t : tests) {
        int before = g_failed;
        t.fn();
        std::printf("  [%s] %s\n", g_failed == before ? "OK  " : "FAIL", t.name);
    }
    std::printf("--- %d kiem tra, %d that bai ---\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
