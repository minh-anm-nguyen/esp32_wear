// Tests for the touch state machine and its transition queue.
//   ./run_tests.sh
//
// The test that justifies the whole class is testFastTapBetweenTwoPolls(): a
// tap that goes down and up between two LVGL polls. With a latest-state cache
// the UI sees "released" and the tap never happened -- intermittently, which
// makes it the hardest kind of bug to reproduce on hardware. Here it is one
// deterministic assertion on a laptop.
#include "touch_tracker.hpp"

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

// ------------------------------------------------------------------ helpers

static ParsedFrame contact(uint16_t x, uint16_t y, EventFlag ev = EventFlag::Contact)
{
    ParsedFrame f{};
    f.status  = FrameStatus::Valid;
    f.fingers = 1;
    f.event   = ev;
    f.rawX    = x;
    f.rawY    = y;
    return f;
}

static ParsedFrame lifted()
{
    ParsedFrame f{};
    f.status  = FrameStatus::NoFinger;
    f.fingers = 0;
    f.event   = EventFlag::Up;
    return f;
}

static ParsedFrame corrupt(FrameStatus s)
{
    ParsedFrame f{};
    f.status = s;
    return f;
}

static TouchPoint at(int16_t x, int16_t y)
{
    TouchPoint p{};
    p.x     = x;
    p.y     = y;
    p.valid = true;
    return p;
}

// Drains the queue the way TouchLvglAdapter will, and reports what it saw.
struct Drained {
    int      downs{0};
    int      moves{0};
    int      ups{0};
    int      syntheticUps{0};
    int16_t  lastX{-1};
    int16_t  lastY{-1};
    uint32_t firstSeq{0};
    uint32_t lastSeq{0};
    int      total{0};
};

static Drained drain(TouchTracker& t)
{
    Drained         d{};
    TouchTransition tr{};
    while (t.pop(tr)) {
        ++d.total;
        if (d.firstSeq == 0) d.firstSeq = tr.sequence;
        d.lastSeq = tr.sequence;
        d.lastX   = tr.x;
        d.lastY   = tr.y;
        switch (tr.kind) {
        case TransitionKind::Down: ++d.downs; break;
        case TransitionKind::Move: ++d.moves; break;
        case TransitionKind::Up:
            ++d.ups;
            if (tr.synthetic) ++d.syntheticUps;
            break;
        }
    }
    return d;
}

// -------------------------------------------------------------------- tests

static void testDownMoveUp()
{
    TouchTracker t;

    t.onFrame(contact(10, 20, EventFlag::Down), at(10, 20), 1000);
    t.onFrame(contact(11, 21), at(11, 21), 1010);
    t.onFrame(contact(12, 22), at(12, 22), 1020);
    t.onFrame(lifted(), TouchPoint{}, 1030);

    const Drained d = drain(t);
    EXPECT(d.downs == 1, "downs=%d", d.downs);
    EXPECT(d.ups == 1, "ups=%d", d.ups);
    EXPECT(d.syntheticUps == 0, "release that phai KHONG synthetic");
    EXPECT(d.moves >= 1, "moves=%d", d.moves);

    // Released at the last known point, not at (0,0): LVGL decides what was
    // clicked from where the release landed.
    EXPECT(d.lastX == 12 && d.lastY == 22, "nha tay tai (%d,%d)", d.lastX, d.lastY);

    // Sequence numbers are monotonic and gapless within one gesture.
    EXPECT(d.lastSeq - d.firstSeq + 1 == static_cast<uint32_t>(d.total),
           "sequence co lo hong: first=%u last=%u total=%d",
           d.firstSeq, d.lastSeq, d.total);
}

// THE reason this class exists.
static void testFastTapBetweenTwoPolls()
{
    TouchTracker t;

    // Both halves of the tap arrive before the UI looks even once.
    t.onFrame(contact(50, 60, EventFlag::Down), at(50, 60), 2000);
    t.onFrame(lifted(), TouchPoint{}, 2008);

    // Now the UI polls. A latest-state cache would say "released" here and the
    // tap would be gone.
    const Drained d = drain(t);
    EXPECT(d.downs == 1, "tap nhanh phai giu duoc Down: downs=%d", d.downs);
    EXPECT(d.ups == 1, "tap nhanh phai giu duoc Up: ups=%d", d.ups);

    // And in the right ORDER: Down before Up, or LVGL sees a release it never
    // had a press for.
    TouchTracker t2;
    t2.onFrame(contact(50, 60, EventFlag::Down), at(50, 60), 2000);
    t2.onFrame(lifted(), TouchPoint{}, 2008);
    TouchTransition tr{};
    EXPECT(t2.pop(tr) && tr.kind == TransitionKind::Down, "transition dau phai la Down");
    EXPECT(t2.pop(tr) && tr.kind == TransitionKind::Up, "transition sau phai la Up");
    EXPECT(!t2.pop(tr), "khong con gi nua");
}

static void testSyntheticRelease()
{
    TouchTracker  t;
    TrackerConfig cfg{};
    cfg.releaseTimeoutMs = 120;
    t.configure(cfg);

    t.onFrame(contact(10, 10, EventFlag::Down), at(10, 10), 5000);

    // The finger lifted but the chip never said so: no more frames arrive.
    t.tick(5100);
    EXPECT(t.snapshot().pressed, "truoc timeout van phai la pressed");

    t.tick(5120);
    EXPECT(!t.snapshot().pressed, "sau timeout phai nha ra");

    const Drained d = drain(t);
    EXPECT(d.ups == 1 && d.syntheticUps == 1, "phai co dung 1 synthetic Up: ups=%d syn=%d",
           d.ups, d.syntheticUps);
    EXPECT(t.syntheticUpCount() == 1, "counter=%u", t.syntheticUpCount());

    // And it must not fire twice.
    t.tick(6000);
    EXPECT(t.queued() == 0, "khong duoc phat them Up nao nua");
}

// A run of corrupt frames during a contact must NOT refresh the contact clock.
// If it did, a chip babbling 0xFF would hold a widget down indefinitely.
static void testCorruptFramesDoNotHoldThePress()
{
    TouchTracker  t;
    TrackerConfig cfg{};
    cfg.releaseTimeoutMs = 100;
    t.configure(cfg);

    t.onFrame(contact(10, 10, EventFlag::Down), at(10, 10), 1000);
    drain(t);

    for (uint32_t ms = 1010; ms < 1100; ms += 10) {
        t.onFrame(corrupt(FrameStatus::AllOnes), TouchPoint{}, ms);
    }
    EXPECT(t.queued() == 0, "frame hong khong duoc sinh transition nao");

    t.tick(1100);
    const Drained d = drain(t);
    EXPECT(d.syntheticUps == 1, "phai nha ra du frame hong lien tuc: syn=%d",
           d.syntheticUps);

    // Every corrupt status behaves the same way.
    const FrameStatus bad[] = {FrameStatus::AllOnes, FrameStatus::BadFingerCount,
                               FrameStatus::ReservedEvent, FrameStatus::OutOfRange};
    for (FrameStatus s : bad) {
        TouchTracker t2;
        t2.onFrame(corrupt(s), TouchPoint{}, 100);
        EXPECT(t2.queued() == 0, "%s khong duoc thanh transition", toString(s));
        EXPECT(!t2.snapshot().pressed, "%s khong duoc thanh pressed", toString(s));
    }
}

// A drag produces a transition roughly every 10 ms. Without collapsing, an
// eight-slot queue overflows in under a tenth of a second.
static void testMoveCoalescing()
{
    TouchTracker t;

    t.onFrame(contact(0, 0, EventFlag::Down), at(0, 0), 1000);
    for (int i = 1; i <= 50; ++i) {
        t.onFrame(contact(static_cast<uint16_t>(i), 0), at(static_cast<int16_t>(i), 0),
                  static_cast<uint32_t>(1000 + i * 10));
    }

    EXPECT(t.queued() == 2, "Down + mot Move gop lai = 2, duoc %u",
           static_cast<unsigned>(t.queued()));
    EXPECT(t.overflowCount() == 0, "gop Move thi khong duoc tran: %u", t.overflowCount());
    EXPECT(t.coalescedMoveCount() == 49, "so lan gop=%u", t.coalescedMoveCount());

    const Drained d = drain(t);
    EXPECT(d.downs == 1 && d.moves == 1, "downs=%d moves=%d", d.downs, d.moves);
    EXPECT(d.lastX == 50, "Move gop phai giu vi tri MOI NHAT, duoc x=%d", d.lastX);
}

// When the queue really does overflow, an Up must survive. A dropped Move
// costs a frame of a drag; a dropped Up leaves the UI stuck.
static void testOverflowNeverDropsAnUp()
{
    TouchTracker t;

    // Fill with alternating taps so the queue holds Downs and Ups, never two
    // adjacent Moves that could collapse.
    for (int i = 0; i < 12; ++i) {
        t.onFrame(contact(static_cast<uint16_t>(i), 0, EventFlag::Down),
                  at(static_cast<int16_t>(i), 0), static_cast<uint32_t>(1000 + i * 2));
        t.onFrame(lifted(), TouchPoint{}, static_cast<uint32_t>(1001 + i * 2));
    }

    EXPECT(t.overflowCount() > 0, "phai co tran that su de bai test co nghia");
    EXPECT(t.queued() <= TouchTracker::kCapacity, "queued=%u",
           static_cast<unsigned>(t.queued()));

    const Drained d = drain(t);
    EXPECT(d.ups > 0, "phai con Up trong hang doi");

    // The end state is what matters: not stuck pressed.
    EXPECT(!t.snapshot().pressed, "sau khi tran khong duoc ket o trang thai pressed");
}

// A Down arriving while already pressed means an Up went missing. Emitting the
// Down alone would give LVGL one impossibly long drag across the screen.
static void testMissedUpIsClosedOut()
{
    TouchTracker t;

    t.onFrame(contact(10, 10, EventFlag::Down), at(10, 10), 1000);
    t.onFrame(contact(200, 200, EventFlag::Down), at(200, 200), 1050);

    EXPECT(t.missedUpCount() == 1, "missedUp=%u", t.missedUpCount());

    TouchTransition tr{};
    EXPECT(t.pop(tr) && tr.kind == TransitionKind::Down && tr.x == 10, "Down dau tien");
    EXPECT(t.pop(tr) && tr.kind == TransitionKind::Up && tr.synthetic,
           "phai chen mot synthetic Up");
    EXPECT(tr.x == 10 && tr.y == 10, "Up phai o vi tri CU (%d,%d)", tr.x, tr.y);
    EXPECT(t.pop(tr) && tr.kind == TransitionKind::Down && tr.x == 200, "Down thu hai");
}

// Some firmware reports finger count 1 together with an Up event flag. A
// missed Move costs one frame; a missed Up leaves a widget held down.
static void testEventUpWinsOverFingerCount()
{
    TouchTracker t;

    t.onFrame(contact(10, 10, EventFlag::Down), at(10, 10), 1000);
    drain(t);

    ParsedFrame f = contact(10, 10, EventFlag::Up);
    f.fingerEventMismatch = true;
    t.onFrame(f, at(10, 10), 1010);

    EXPECT(t.eventUpWithFingerCount() == 1, "counter=%u", t.eventUpWithFingerCount());
    EXPECT(!t.snapshot().pressed, "event=Up phai thang finger count");

    const Drained d = drain(t);
    EXPECT(d.ups == 1 && d.syntheticUps == 0, "day la Up THAT: ups=%d syn=%d",
           d.ups, d.syntheticUps);

    // And the opposite policy is honoured when asked for.
    TouchTracker  t2;
    TrackerConfig cfg{};
    cfg.trustEventUp = false;
    t2.configure(cfg);
    t2.onFrame(contact(10, 10, EventFlag::Down), at(10, 10), 1000);
    drain(t2);
    t2.onFrame(f, at(10, 10), 1010);
    EXPECT(t2.snapshot().pressed, "trustEventUp=false thi van giu pressed");
    EXPECT(t2.eventUpWithFingerCount() == 1, "van phai dem");
}

// The manager calls this before Fault or stop. LVGL must never be left holding
// a widget down because the driver went away.
static void testCancelForcesRelease()
{
    TouchTracker t;

    t.onFrame(contact(10, 10, EventFlag::Down), at(10, 10), 1000);
    drain(t);

    t.cancel(1234);

    const Drained d = drain(t);
    EXPECT(d.ups == 1 && d.syntheticUps == 1, "cancel phai phat synthetic Up");
    EXPECT(!t.snapshot().pressed, "sau cancel phai la released");

    // Idempotent: a second cancel on an already-released tracker emits nothing.
    t.cancel(1300);
    EXPECT(t.queued() == 0, "cancel lan hai khong duoc phat gi");
}

// Diagnostics must be able to read state without stealing events from the UI.
static void testSnapshotDoesNotConsume()
{
    TouchTracker t;

    t.onFrame(contact(33, 44, EventFlag::Down), at(33, 44), 1000);

    const TouchSnapshot s1 = t.snapshot();
    const TouchSnapshot s2 = t.snapshot();
    EXPECT(s1.pressed && s1.x == 33 && s1.y == 44, "snapshot (%d,%d)", s1.x, s1.y);
    EXPECT(s2.sequence == s1.sequence, "doc snapshot khong duoc lam thay doi gi");
    EXPECT(t.queued() == 1, "snapshot khong duoc tieu thu transition: queued=%u",
           static_cast<unsigned>(t.queued()));
}

// A millisecond counter wraps every 49 days. Unsigned subtraction handles it;
// a signed comparison would not, and the watch would stop releasing.
static void testTimeoutSurvivesCounterWrap()
{
    TouchTracker  t;
    TrackerConfig cfg{};
    cfg.releaseTimeoutMs = 100;
    t.configure(cfg);

    const uint32_t nearMax = 0xFFFFFFF0u;
    t.onFrame(contact(10, 10, EventFlag::Down), at(10, 10), nearMax);
    drain(t);

    t.tick(nearMax + 50);  // wraps past zero
    EXPECT(t.snapshot().pressed, "50 ms chua den han");

    t.tick(nearMax + 120);
    EXPECT(!t.snapshot().pressed, "qua han thi phai nha du bo dem da tran");
}

int main()
{
    std::printf("touch_tracker\n");

    testDownMoveUp();
    testFastTapBetweenTwoPolls();
    testSyntheticRelease();
    testCorruptFramesDoNotHoldThePress();
    testMoveCoalescing();
    testOverflowNeverDropsAnUp();
    testMissedUpIsClosedOut();
    testEventUpWinsOverFingerCount();
    testCancelForcesRelease();
    testSnapshotDoesNotConsume();
    testTimeoutSurvivesCounterWrap();

    std::printf("  %d kiem tra, %d that bai\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
