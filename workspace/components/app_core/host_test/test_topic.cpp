// Tests for the topic generation protocol. Pure logic, runs on a PC.
//   ./run_tests.sh
//
// The bugs this file exists to prevent, all of which are quiet:
//   1. First read returns nothing -> an app entering the foreground shows a
//      blank field until the next publish. On a 1/60 Hz topic like the clock
//      that is a blank minute.
//   2. Two readers sharing staleness -> the second app to poll sees nothing,
//      because the first one "used up" the update.
//   3. Generation wrapping to 0 -> "never published" and "wrapped" collide and
//      every reader goes permanently stale. ~497 days at 100 Hz.
//   4. Coalescing losing the LATEST value instead of the intermediate ones.
#include "topic.hpp"

#include <cstdio>
#include <cstdint>

using namespace core;

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

// A plausible topic payload: small, POD, no pointers.
struct BatteryState {
    uint8_t  percent{};
    bool     charging{};
    uint32_t timestampMs{};
};

// ------------------------------------------------------------ empty behaviour

static void testEmptyTopicYieldsNothing()
{
    Topic<BatteryState> t;
    BatteryState        v{};
    Cursor              c{};

    EXPECT(!t.hasValue(), "topic chua publish thi hasValue()==false");
    EXPECT(t.generation() == 0, "generation() ban dau phai la 0");
    EXPECT(!t.read(v, c), "read() tren topic rong phai tra false");
    EXPECT(!t.peek(v), "peek() tren topic rong phai tra false");
    EXPECT(c.lastSeen == 0, "read() that bai khong duoc dong cursor");
}

// -------------------------------------------------------------- the first read

static void testFirstReadReturnsLatest()
{
    Topic<BatteryState> t;
    t.publish({47, false, 1000});

    // A brand new reader -- an app that just entered the foreground -- must get
    // the CURRENT value on its very first poll, not wait for the next publish.
    Cursor       c{};
    BatteryState v{};
    EXPECT(t.read(v, c), "reader moi phai nhan duoc gia tri hien tai ngay");
    EXPECT(v.percent == 47, "gia tri dung, thuc te %u", v.percent);
    EXPECT(c.lastSeen == 1, "cursor phai tien toi generation 1");

    EXPECT(!t.read(v, c), "doc lai khi khong co gi moi phai tra false");
}

static void testLateReaderSkipsHistory()
{
    Topic<BatteryState> t;
    for (uint8_t i = 0; i < 5; ++i) {
        t.publish({i, false, i});
    }

    // Joining late gives the newest value, not the oldest and not five reads.
    // This is the whole point of "state, not events".
    Cursor       c{};
    BatteryState v{};
    EXPECT(t.read(v, c), "reader vao muon van doc duoc");
    EXPECT(v.percent == 4, "phai nhan gia tri MOI NHAT, thuc te %u", v.percent);
    EXPECT(!t.read(v, c), "va sau do khong con gi moi");
}

// ------------------------------------------------------------ reader isolation

static void testReadersAreIndependent()
{
    Topic<BatteryState> t;
    Cursor              slow{}, fast{};
    BatteryState        v{};

    t.publish({10, false, 1});

    EXPECT(t.read(v, fast) && v.percent == 10, "reader nhanh doc lan 1");
    EXPECT(!t.read(v, fast), "reader nhanh khong con gi moi");

    // The slow reader has not looked yet. The fast reader must not have
    // consumed the update on its behalf.
    EXPECT(t.read(v, slow), "reader cham VAN phai doc duoc");
    EXPECT(v.percent == 10, "reader cham nhan dung gia tri, thuc te %u",
           v.percent);

    t.publish({20, true, 2});
    EXPECT(t.read(v, fast) && v.percent == 20, "reader nhanh thay gia tri moi");
    EXPECT(t.read(v, slow) && v.percent == 20, "reader cham cung thay");
}

// ---------------------------------------------------------------- coalescing

static void testCoalescingKeepsTheNewest()
{
    Topic<BatteryState> t;
    Cursor              c{};
    BatteryState        v{};

    t.publish({1, false, 1});
    EXPECT(t.read(v, c), "doc gia tri dau");

    // 200 publishes between two reads -- an IMU-rate producer against a 30 fps
    // consumer. Exactly one read, and it must yield the LAST value, not the
    // first one after the cursor.
    for (uint32_t i = 2; i <= 201; ++i) {
        t.publish({static_cast<uint8_t>(i % 256), false, i});
    }
    EXPECT(t.read(v, c), "sau 200 publish phai co gi do moi");
    EXPECT(v.timestampMs == 201, "phai la mau CUOI, thuc te %u", v.timestampMs);
    EXPECT(!t.read(v, c), "doc lan hai khong con gi");
}

static void testGenerationCountsPublishes()
{
    Topic<BatteryState> t;
    for (uint32_t i = 0; i < 7; ++i) {
        t.publish({0, false, i});
    }
    EXPECT(t.generation() == 7, "generation()==7, thuc te %u", t.generation());
}

// ------------------------------------------------------------------- peek

static void testPeekDoesNotMoveCursor()
{
    Topic<BatteryState> t;
    Cursor              c{};
    BatteryState        v{};

    t.publish({33, false, 5});

    EXPECT(t.peek(v), "peek() sau publish phai thanh cong");
    EXPECT(v.percent == 33, "peek() tra dung gia tri");
    EXPECT(c.lastSeen == 0, "peek() KHONG duoc dong cursor");

    // A screen that peeked while being built must still get the update on its
    // first real poll, or it will never subscribe to anything.
    EXPECT(t.read(v, c), "read() sau peek() van phai tra true");
}

// ------------------------------------------------------------------ wraparound

// A topic that lets the test drive the counter to the edge without publishing
// four billion times.
struct Tiny {
    uint32_t v{};
};

// Exercises the REAL increment out of topic.hpp, not a copy of it. Reaching
// into detail:: is deliberate: publish() cannot be driven to 2^32 in a test, so
// the rule is named there and checked here.
static void testWrapArithmeticNeverYieldsZero()
{
    EXPECT(detail::nextGeneration(0) == 1, "publish dau tien -> 1");
    EXPECT(detail::nextGeneration(1) == 2, "buoc thuong");
    EXPECT(detail::nextGeneration(0xFFFFFFFEu) == 0xFFFFFFFFu, "buoc truoc dinh");

    // The one that matters: 0xFFFFFFFF + 1 would be 0, which collides with
    // "never published" and makes EVERY reader permanently stale.
    EXPECT(detail::nextGeneration(0xFFFFFFFFu) == 1, "wrap phai nhay qua 0, thuc te %u",
           detail::nextGeneration(0xFFFFFFFFu));
}

// And the same rule observed through the public API, so the two cannot drift.
static void testPublishNeverLeavesGenerationAtZero()
{
    Topic<Tiny> t;
    Cursor      c{};
    Tiny        out{};

    for (uint32_t i = 1; i <= 4; ++i) {
        t.publish({i});
        EXPECT(t.generation() == i, "generation theo sat so lan publish");
        EXPECT(t.generation() != 0, "generation khong bao gio la 0 sau publish");
    }
    EXPECT(t.read(out, c) && out.v == 4, "van doc duoc gia tri moi nhat");
}

// ------------------------------------------------------------- payload fidelity

static void testPayloadSurvivesIntact()
{
    struct Wide {
        uint32_t a;
        int16_t  b;
        float    c;
        bool     d;
        uint8_t  e[3];
    };

    Topic<Wide> t;
    Cursor      cur{};
    Wide        out{};

    const Wide sent{0xDEADBEEFu, -1234, 3.5f, true, {7, 8, 9}};
    t.publish(sent);
    EXPECT(t.read(out, cur), "doc duoc");
    EXPECT(out.a == 0xDEADBEEFu && out.b == -1234 && out.c == 3.5f
               && out.d && out.e[0] == 7 && out.e[1] == 8 && out.e[2] == 9,
           "moi truong phai di qua nguyen ven");
}

// ------------------------------------------------------- lock policy is called

// Proves the policy is actually invoked, and paired. A policy that is declared
// but never called would make every cross-task topic silently unsynchronised --
// and it would pass every other test in this file.
struct CountingLock {
    static int enters;
    static int exits;
    void       enter() const { ++enters; }
    void       exit() const { ++exits; }
};
int CountingLock::enters = 0;
int CountingLock::exits  = 0;

static void testLockPolicyIsEnteredAndExited()
{
    CountingLock::enters = 0;
    CountingLock::exits  = 0;

    Topic<Tiny, CountingLock> t;
    Cursor                    c{};
    Tiny                      v{};

    t.publish({1});
    t.read(v, c);
    t.peek(v);
    t.hasValue();
    t.generation();

    EXPECT(CountingLock::enters == 5, "5 thao tac -> 5 lan enter, thuc te %d",
           CountingLock::enters);
    EXPECT(CountingLock::enters == CountingLock::exits,
           "enter/exit phai can bang: %d vs %d", CountingLock::enters,
           CountingLock::exits);
}

static void testEarlyReturnStillUnlocks()
{
    CountingLock::enters = 0;
    CountingLock::exits  = 0;

    Topic<Tiny, CountingLock> t;
    Cursor                    c{};
    Tiny                      v{};

    // Both early-return paths: never published, and nothing new.
    t.read(v, c);       // g == 0
    t.publish({1});
    t.read(v, c);       // succeeds
    t.read(v, c);       // g == cursor.lastSeen

    // The whole point: an early return that forgets to unlock deadlocks the
    // system on target and is invisible on host without this check.
    EXPECT(CountingLock::enters == CountingLock::exits,
           "duong thoat som van phai exit: %d vs %d", CountingLock::enters,
           CountingLock::exits);
}

int main()
{
    std::printf("topic\n");

    testEmptyTopicYieldsNothing();
    testFirstReadReturnsLatest();
    testLateReaderSkipsHistory();
    testReadersAreIndependent();
    testCoalescingKeepsTheNewest();
    testGenerationCountsPublishes();
    testPeekDoesNotMoveCursor();
    testWrapArithmeticNeverYieldsZero();
    testPublishNeverLeavesGenerationAtZero();
    testPayloadSurvivesIntact();
    testLockPolicyIsEnteredAndExited();
    testEarlyReturnStillUnlocks();

    std::printf("  %d kiem tra, %d that bai\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
