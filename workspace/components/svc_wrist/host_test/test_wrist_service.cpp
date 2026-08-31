// Tests for the first service: IMU samples in, "wrist is raised" state out.
//   ./run_tests.sh
//
// This is the seam between an EVENT source (components/motion) and a STATE
// consumer (an app reading a topic), so the bugs live at that boundary:
//   1. State published only on the event -> an app entering the foreground
//      later reads nothing and renders a blank.
//   2. Event forwarded BEFORE the topic is published -> whatever the event
//      wakes reads the state from before the raise. One-frame flicker,
//      horrible to attribute.
//   3. The service silently stopping after the first raise.
#include "wrist_service.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace svc;

static int g_checks = 0;
static int g_failed = 0;

#define EXPECT(cond, ...)                                       \
    do {                                                        \
        ++g_checks;                                             \
        if (!(cond)) {                                          \
            ++g_failed;                                         \
            std::printf("  FAIL  %s:%d  ", __FILE__, __LINE__); \
            std::printf(__VA_ARGS__);                           \
            std::printf("\n");                                  \
        }                                                       \
    } while (0)

// Same pose synthesis the components/motion tests use, so a change in the
// gesture thresholds shows up in both places rather than only one.
//   armPitch    -90 = hanging by the side, +90 = straight up
//   screenPitch +90 = glass at the sky, 0 = glass vertical
static sensors::Sample pose(float armPitchDeg, float screenPitchDeg)
{
    const float k = 0.017453292519943295f;
    const float y = std::sin(armPitchDeg * k);
    const float z = std::sin(screenPitchDeg * k);
    float       rest = 1.0f - y * y - z * z;
    if (rest < 0.0f) {
        rest = 0.0f;
    }

    sensors::Sample s{};
    s.accelG[0] = std::sqrt(rest);
    s.accelG[1] = y;
    s.accelG[2] = z;
    return s;
}

static sensors::Sample armHanging() { return pose(-90.0f, 0.0f); }
static sensors::Sample readingPose() { return pose(0.0f, 85.0f); }

// Feeds a pose for `ms` at the real 62.5 Hz ACTIVITY rate.
struct Driver {
    WristService svc;
    uint32_t     t{0};

    void hold(const sensors::Sample& s, uint32_t ms)
    {
        for (uint32_t e = 0; e < ms; e += 16) {
            svc.onSample(s, t);
            t += 16;
        }
    }
};

// ------------------------------------------------------------- the basic path

static void testRaiseAndLowerBecomeState()
{
    Driver     d;
    core::Cursor c{};
    WristState st{};

    EXPECT(!d.svc.state().hasValue(), "truoc khi co cu chi, topic phai rong");

    d.hold(armHanging(), 400);
    d.hold(readingPose(), 600);

    EXPECT(d.svc.state().read(st, c), "sau khi nang tay phai co state");
    EXPECT(st.raised, "phai la raised=true");
    EXPECT(st.raiseCount == 1, "raiseCount==1, thuc te %u", st.raiseCount);
    EXPECT(st.sinceMs != 0, "phai co moc thoi gian");

    d.hold(armHanging(), 600);
    EXPECT(d.svc.state().read(st, c), "sau khi ha tay phai co state moi");
    EXPECT(!st.raised, "phai la raised=false");
}

static void testLateReaderSeesCurrentState()
{
    // The bug this prevents: an app that enters the foreground AFTER the raise
    // has missed the event. If the service only forwarded events it would have
    // nothing to render. State must survive the moment it was created.
    Driver d;
    d.hold(armHanging(), 400);
    d.hold(readingPose(), 600);

    // A brand-new reader, arriving long after the gesture.
    core::Cursor late{};
    WristState   st{};
    EXPECT(d.svc.state().read(st, late), "reader den muon VAN phai doc duoc");
    EXPECT(st.raised, "va phai thay trang thai HIEN TAI, khong phai trang thai rong");
}

static void testRepeatedGesturesKeepWorking()
{
    Driver d;
    for (uint32_t i = 1; i <= 5; ++i) {
        d.hold(armHanging(), 600);
        d.hold(readingPose(), 600);

        WristState st{};
        EXPECT(d.svc.state().peek(st), "vong %u: co state", i);
        EXPECT(st.raised, "vong %u: dang raised", i);
        EXPECT(st.raiseCount == i, "vong %u: raiseCount==%u, thuc te %u", i, i,
               st.raiseCount);
    }
}

// -------------------------------------------------------------- ordering rule

namespace {

struct OrderProbe {
    WristService*            svc{nullptr};
    std::vector<bool>        stateSeenAtCallback;
    int                      calls{0};
};

void onEventProbe(void* ctx, motion::MotionEvent ev, uint32_t)
{
    auto* p = static_cast<OrderProbe*>(ctx);
    ++p->calls;

    // Read the topic from INSIDE the event callback. If publish() happened
    // first -- as it must -- this already reflects the new state.
    WristState st{};
    p->svc->state().peek(st);
    if (ev == motion::MotionEvent::WRIST_RAISE) {
        p->stateSeenAtCallback.push_back(st.raised);
    }
}

}  // namespace

static void testStateIsPublishedBeforeEventIsForwarded()
{
    Driver     d;
    OrderProbe probe;
    probe.svc = &d.svc;
    d.svc.setEventCallback(onEventProbe, &probe);

    d.hold(armHanging(), 400);
    d.hold(readingPose(), 600);

    EXPECT(probe.calls >= 1, "callback phai duoc goi");
    EXPECT(probe.stateSeenAtCallback.size() == 1, "dung mot RAISE, thuc te %zu",
           probe.stateSeenAtCallback.size());

    // THE test of this file. Whatever the raise wakes must not read stale state.
    EXPECT(!probe.stateSeenAtCallback.empty() && probe.stateSeenAtCallback[0],
           "callback RAISE phai thay raised=true -- publish() phai chay TRUOC "
           "khi forward su kien");
}

static void testEventCallbackIsOptional()
{
    // No callback set: the topic must still work. A service whose state depends
    // on somebody having subscribed is a service with a hidden prerequisite.
    Driver d;
    d.hold(armHanging(), 400);
    d.hold(readingPose(), 600);

    WristState st{};
    EXPECT(d.svc.state().peek(st) && st.raised,
           "topic phai chay ke ca khi khong ai dang ky callback");
}

// ------------------------------------------------------- no spurious publishes

static void testStillArmPublishesNothing()
{
    Driver d;
    d.hold(armHanging(), 3000);  // three seconds of nothing happening

    // A topic that republishes on every sample would make every reader wake up
    // 62 times a second to be told nothing changed -- which is precisely the
    // cost coalescing exists to avoid.
    EXPECT(!d.svc.state().hasValue(),
           "tay yen thi khong duoc publish gi, generation=%u",
           d.svc.state().generation());
}

static void testGenerationTracksGesturesNotSamples()
{
    Driver d;
    d.hold(armHanging(), 400);
    d.hold(readingPose(), 600);
    d.hold(armHanging(), 600);

    // Two gestures over ~1600 ms = ~100 samples. Two publishes, not 100.
    EXPECT(d.svc.state().generation() == 2,
           "2 cu chi -> generation 2, thuc te %u", d.svc.state().generation());
}

int main()
{
    std::printf("wrist_service\n");

    testRaiseAndLowerBecomeState();
    testLateReaderSeesCurrentState();
    testRepeatedGesturesKeepWorking();
    testStateIsPublishedBeforeEventIsForwarded();
    testEventCallbackIsOptional();
    testStillArmPublishesNothing();
    testGenerationTracksGesturesNotSamples();

    std::printf("  %d kiem tra, %d that bai\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
