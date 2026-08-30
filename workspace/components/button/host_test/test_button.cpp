// Tests for the pure logic layer, run on a PC with g++. No chip, no ESP-IDF.
//   ./run_tests.sh
#include "button.hpp"

#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

using namespace button;

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

static const char* name(ButtonEvent e)
{
    switch (e) {
    case ButtonEvent::NONE:               return "NONE";
    case ButtonEvent::PRESS_DOWN:         return "PRESS_DOWN";
    case ButtonEvent::CLICK:              return "CLICK";
    case ButtonEvent::DOUBLE_CLICK:       return "DOUBLE_CLICK";
    case ButtonEvent::DOUBLE_CLICK_HOLD:  return "DOUBLE_CLICK_HOLD";
    case ButtonEvent::LONG_PRESS:         return "LONG_PRESS";
    case ButtonEvent::LONG_PRESS_RELEASE: return "LONG_PRESS_RELEASE";
    }
    return "?";
}

static std::string join(const std::vector<ButtonEvent>& v)
{
    if (v.empty()) return "(rong)";
    std::string s;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) s += " , ";
        s += name(v[i]);
    }
    return s;
}

// Simulates one button: feeds a signal described as (duration, level) segments.
class Sim {
public:
    Sim(const ButtonBehavior& cfg, uint32_t pollMs, uint32_t startMs = 0)
        : btn_(cfg, debounceCountFor(cfg.debounceMs, pollMs)),
          poll_(pollMs),
          t_(startMs)
    {
    }

    // Force the debounce cycle count directly, bypassing the debounceMs conversion.
    Sim(const ButtonBehavior& cfg, uint32_t pollMs, uint8_t forcedCnt, uint32_t startMs)
        : btn_(cfg, forcedCnt), poll_(pollMs), t_(startMs)
    {
    }

    void feed(bool level, uint32_t durMs)
    {
        for (uint32_t d = 0; d < durMs; d += poll_) {
            ButtonEvent e = btn_.update(level, t_);
            if (e != ButtonEvent::NONE) {
                events.push_back(e);
                stamps.push_back(t_);
                pressStamps.push_back(btn_.pressTimestampMs());
            }
            t_ += poll_;
        }
    }

    // Worst-case chatter: flip the level on every single poll cycle.
    void chatter(uint32_t cycles)
    {
        for (uint32_t i = 0; i < cycles; ++i) {
            feed(i % 2 == 0, poll_);
        }
    }

    Button&  btn() { return btn_; }
    uint32_t now() const { return t_; }

    std::vector<ButtonEvent> events;
    std::vector<uint32_t>    stamps;
    std::vector<uint32_t>    pressStamps;

private:
    Button   btn_;
    uint32_t poll_;
    uint32_t t_;
};

static ButtonBehavior baseCfg()
{
    ButtonBehavior c{};
    c.enableDoubleClick = true;
    c.longPressMs       = 800;
    c.doubleClickMs     = 250;
    c.debounceMs        = 20;
    return c;
}

static bool seq(const std::vector<ButtonEvent>& got,
                const std::vector<ButtonEvent>& want)
{
    return got == want;
}

// ------------------------------------------------------------------ test cases

static void test_single_click()
{
    Sim s(baseCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 400);
    EXPECT(seq(s.events, {ButtonEvent::CLICK}), "mong CLICK, nhan: %s",
           join(s.events).c_str());
}

static void test_double_click()
{
    Sim s(baseCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 120);
    s.feed(true, 60);
    s.feed(false, 300);
    EXPECT(seq(s.events, {ButtonEvent::DOUBLE_CLICK}),
           "mong DOUBLE_CLICK don doc, nhan: %s", join(s.events).c_str());
}

static void test_long_press()
{
    Sim s(baseCfg(), 10);
    s.feed(true, 1000);
    s.feed(false, 200);
    EXPECT(seq(s.events, {ButtonEvent::LONG_PRESS, ButtonEvent::LONG_PRESS_RELEASE}),
           "mong LONG_PRESS + LONG_PRESS_RELEASE, nhan: %s", join(s.events).c_str());
}

static void test_chatter_rejected()
{
    ButtonBehavior c = baseCfg();
    Sim s(c, 10);            // debounceMs=20, poll=10 -> maxCnt=2
    s.chatter(40);           // level flips every cycle: never 2 consecutive agreeing reads
    s.feed(false, 400);
    EXPECT(s.events.empty(), "nhieu lien tuc khong duoc sinh su kien, nhan: %s",
           join(s.events).c_str());
}

static void test_double_click_disabled()
{
    ButtonBehavior c   = baseCfg();
    c.enableDoubleClick = false;
    Sim s(c, 10);
    s.feed(true, 60);
    s.feed(false, 200);
    EXPECT(seq(s.events, {ButtonEvent::CLICK}), "mong CLICK ngay khi nha, nhan: %s",
           join(s.events).c_str());
    // No waiting window any more, so CLICK must arrive far earlier than with
    // double click enabled.
    EXPECT(!s.stamps.empty() && s.stamps[0] < 150,
           "CLICK phai phat ngay khi nha, timestamp=%u",
           s.stamps.empty() ? 0u : s.stamps[0]);
}

static void test_uint32_wrap()
{
    Sim s(baseCfg(), 10, /*startMs=*/0xFFFFFF00u);
    s.feed(true, 60);
    s.feed(false, 400);
    EXPECT(seq(s.events, {ButtonEvent::CLICK}),
           "gan diem tran uint32 van phai ra CLICK, nhan: %s", join(s.events).c_str());
}

static void test_long_press_wrap()
{
    // Press starts before the wrap point, long-press threshold falls after it.
    Sim s(baseCfg(), 10, /*startMs=*/0xFFFFFF00u);
    s.feed(true, 1000);
    s.feed(false, 200);
    EXPECT(seq(s.events, {ButtonEvent::LONG_PRESS, ButtonEvent::LONG_PRESS_RELEASE}),
           "long press vat qua diem tran, nhan: %s", join(s.events).c_str());
}

static void test_double_click_then_hold()
{
    Sim s(baseCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 120);
    s.feed(true, 3000);      // hold for 3 seconds after the double click
    s.feed(false, 200);
    EXPECT(seq(s.events, {ButtonEvent::DOUBLE_CLICK}),
           "mac dinh enableDoubleClickHold=false -> WAIT_RELEASE nuot phan duoi, "
           "khong LONG_PRESS ma, khong ca DOUBLE_CLICK_HOLD. nhan: %s",
           join(s.events).c_str());
    EXPECT(s.btn().getState() == ButtonState::IDLE, "phai ve IDLE sau khi nha");
    EXPECT(s.btn().isIdle(), "isIdle() phai true de manager duoc ngu");
}

static void test_edge_beats_timeout()
{
    // Construct the case where the press edge lands on EXACTLY the cycle at which
    // the doubleClickMs window expires.
    // Release at t=60 -> release edge at t=70 -> clickTs=70 -> window ends at t=320.
    // Drive the signal high from t=310 so the press edge lands exactly on t=320.
    Sim s(baseCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 250);      // t advances to 310
    s.feed(true, 60);
    s.feed(false, 300);
    EXPECT(seq(s.events, {ButtonEvent::DOUBLE_CLICK}),
           "canh phai thang timeout, nhan: %s", join(s.events).c_str());
    EXPECT(!s.stamps.empty() && s.stamps[0] == 320,
           "su kien phai roi dung chu ky t=320, thuc te=%u",
           s.stamps.empty() ? 0u : s.stamps[0]);
}

static void test_edge_one_cycle_late()
{
    // Exactly one cycle later -> CLICK, then a brand NEW press.
    Sim s(baseCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 260);
    s.feed(true, 60);
    s.feed(false, 400);
    EXPECT(seq(s.events, {ButtonEvent::CLICK, ButtonEvent::CLICK}),
           "cham mot chu ky -> hai CLICK roi rac, nhan: %s", join(s.events).c_str());
}

static void test_press_timestamp()
{
    {   // CLICK: pressTimestampMs is the press edge, not the emission moment
        Sim s(baseCfg(), 10);
        s.feed(true, 60);
        s.feed(false, 400);
        EXPECT(s.stamps.size() == 1 && s.pressStamps.size() == 1, "phai co dung 1 su kien");
        if (s.stamps.size() == 1) {
            EXPECT(s.stamps[0] == 320, "timestampMs=%u, mong 320", s.stamps[0]);
            EXPECT(s.pressStamps[0] == 10, "pressTimestampMs=%u, mong 10", s.pressStamps[0]);
        }
    }
    {   // DOUBLE_CLICK: keeps the timestamp of the FIRST press
        Sim s(baseCfg(), 10);
        s.feed(true, 60);
        s.feed(false, 120);
        s.feed(true, 60);
        s.feed(false, 300);
        EXPECT(s.pressStamps.size() == 1 && s.pressStamps[0] == 10,
               "DOUBLE_CLICK phai giu moc cu nhan thu nhat (10), thuc te=%u",
               s.pressStamps.empty() ? 0u : s.pressStamps[0]);
    }
    {   // LONG_PRESS and LONG_PRESS_RELEASE are one gesture -> same pressTimestampMs
        Sim s(baseCfg(), 10);
        s.feed(true, 1000);
        s.feed(false, 200);
        EXPECT(s.pressStamps.size() == 2 && s.pressStamps[0] == s.pressStamps[1],
               "hai su kien cung cu chi phai cung pressTimestampMs");
    }
}

static void test_debounce_count_for()
{
    EXPECT(debounceCountFor(20, 10) == 2, "(20,10) = %u", debounceCountFor(20, 10));
    EXPECT(debounceCountFor(20, 5)  == 4, "(20,5)  = %u", debounceCountFor(20, 5));
    EXPECT(debounceCountFor(20, 30) == 1, "(20,30) = %u", debounceCountFor(20, 30));
    EXPECT(debounceCountFor(0, 10)  == 1, "(0,10)  = %u", debounceCountFor(0, 10));
    EXPECT(debounceCountFor(20, 0)  == 1, "(20,0)  = %u", debounceCountFor(20, 0));
    EXPECT(debounceCountFor(100000, 1) == 255, "phai kep tran o 255, = %u",
           debounceCountFor(100000, 1));

    // Ceil invariant: cnt*poll >= debounceMs and < debounceMs + poll
    for (uint32_t d = 1; d <= 60; ++d) {
        for (uint32_t p = 1; p <= 20; ++p) {
            uint32_t n = debounceCountFor(d, p);
            EXPECT(n * p >= d && n * p < d + p, "ceil sai voi (d=%u, p=%u) -> %u", d, p, n);
        }
    }
}

static void test_tick_rate_independence()
{
    // Same debounceMs=20 at two tick rates -> same evidence window.
    ButtonBehavior c = baseCfg();
    EXPECT(debounceCountFor(c.debounceMs, 10) * 10 == 20, "HZ=100: 2 x 10 = 20ms");
    EXPECT(debounceCountFor(c.debounceMs, 5) * 5 == 20,  "HZ=1000: 4 x 5 = 20ms");

    // Behaviour: a glitch shorter than debounceMs is rejected in BOTH setups.
    for (uint32_t poll : {10u, 5u}) {
        Sim s(c, poll);
        s.feed(true, 10);        // 10ms glitch < 20ms debounceMs
        s.feed(false, 400);
        EXPECT(s.events.empty(), "poll=%u: gai 10ms phai bi loai, nhan: %s",
               poll, join(s.events).c_str());
    }
    // And a long enough hold is accepted in BOTH setups.
    for (uint32_t poll : {10u, 5u}) {
        Sim s(c, poll);
        s.feed(true, 60);
        s.feed(false, 400);
        EXPECT(seq(s.events, {ButtonEvent::CLICK}), "poll=%u: giu 60ms phai ra CLICK, nhan: %s",
               poll, join(s.events).c_str());
    }
}

static void test_debounce_zero_forced_to_one()
{
    ButtonBehavior c = baseCfg();
    Sim s(c, 10, /*forcedCnt=*/0, /*startMs=*/0);
    // With maxCnt forced to 1, a pin reporting "released" must yield released.
    s.feed(false, 50);
    EXPECT(!s.btn().isPressed(),
           "maxCnt=0 phai bi ep ve 1; neu khong stableState_ ket cung o 'nhan'");
    EXPECT(s.btn().isIdle(), "phai idle khi khong ai cham vao nut");
}

// ------------------------------------------------------- PRESS_DOWN (opt-in)

static ButtonBehavior pressDownCfg()
{
    ButtonBehavior c      = baseCfg();
    c.enablePressDown   = true;
    return c;
}

static void test_press_down_off_by_default()
{
    EXPECT(!baseCfg().enablePressDown, "enablePressDown phai mac dinh tat");

    Sim s(baseCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 400);
    EXPECT(seq(s.events, {ButtonEvent::CLICK}),
           "mac dinh tat thi chuoi su kien phai y nguyen nhu cu, nhan: %s",
           join(s.events).c_str());
}

static void test_press_down_emitted()
{
    Sim s(pressDownCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 400);
    EXPECT(seq(s.events, {ButtonEvent::PRESS_DOWN, ButtonEvent::CLICK}),
           "mong PRESS_DOWN roi CLICK, nhan: %s", join(s.events).c_str());
}

// The whole point of the event: it must land while the finger is still down,
// not after doubleClickMs has expired.
static void test_press_down_is_immediate()
{
    Sim s(pressDownCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 400);

    EXPECT(s.stamps.size() == 2, "phai co dung 2 su kien, nhan %zu", s.stamps.size());
    if (s.stamps.size() == 2) {
        // debounceMs=20 at a 10ms poll is 2 cycles, so the edge settles at t=10.
        EXPECT(s.stamps[0] <= 20,
               "PRESS_DOWN phai phat ngay sau debounce, nhan t=%u", s.stamps[0]);
        // CLICK still pays the full double click window; that is the delay the
        // press-down event exists to hide.
        EXPECT(s.stamps[1] - s.stamps[0] >= 250,
               "CLICK van phai cho het cua so, chenh lech=%u",
               s.stamps[1] - s.stamps[0]);
    }
}

static void test_press_down_once_per_double_click()
{
    Sim s(pressDownCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 120);
    s.feed(true, 60);
    s.feed(false, 300);
    // The second press starts from WAIT_DOUBLE_CLICK, so it yields DOUBLE_CLICK
    // and NOT a second PRESS_DOWN.
    EXPECT(seq(s.events, {ButtonEvent::PRESS_DOWN, ButtonEvent::DOUBLE_CLICK}),
           "chi mot PRESS_DOWN cho ca cu chi double click, nhan: %s",
           join(s.events).c_str());
}

static void test_press_down_long_press()
{
    Sim s(pressDownCfg(), 10);
    s.feed(true, 1000);
    s.feed(false, 200);
    EXPECT(seq(s.events, {ButtonEvent::PRESS_DOWN, ButtonEvent::LONG_PRESS,
                          ButtonEvent::LONG_PRESS_RELEASE}),
           "mong PRESS_DOWN + LONG_PRESS + RELEASE, nhan: %s", join(s.events).c_str());
}

// PRESS_DOWN sits behind the debounce integrator like every other event, so it
// must not turn contact bounce into a burst of beeps.
static void test_press_down_still_debounced()
{
    Sim s(pressDownCfg(), 10);
    s.chatter(40);
    s.feed(false, 400);
    EXPECT(s.events.empty(), "nhieu lien tuc van khong duoc sinh PRESS_DOWN, nhan: %s",
           join(s.events).c_str());
}

static void test_reset()
{
    Sim s(baseCfg(), 10);
    s.feed(true, 1000);          // still held, LONG_PRESS already emitted
    EXPECT(s.btn().getState() == ButtonState::WAIT_RELEASE_LONG, "phai o WAIT_RELEASE_LONG");
    s.btn().reset();
    EXPECT(s.btn().getState() == ButtonState::IDLE, "reset phai ve IDLE");
    EXPECT(s.btn().isIdle(), "reset phai lam isIdle() true");
    EXPECT(!s.btn().isPressed(), "reset phai xoa stableState_");
    EXPECT(s.btn().pressTimestampMs() == 0, "reset phai xoa pressTimestamp_");
}

static ButtonBehavior holdCfg()
{
    ButtonBehavior c = baseCfg();
    c.enableDoubleClickHold = true;
    return c;
}

static void test_double_click_hold_enabled()
{
    Sim s(holdCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 120);
    s.feed(true, 3000);      // giu 3 giay sau double click
    s.feed(false, 200);
    EXPECT(seq(s.events, {ButtonEvent::DOUBLE_CLICK, ButtonEvent::DOUBLE_CLICK_HOLD}),
           "bat enableDoubleClickHold -> DOUBLE_CLICK roi DOUBLE_CLICK_HOLD. nhan: %s",
           join(s.events).c_str());
    EXPECT(s.btn().getState() == ButtonState::IDLE, "phai ve IDLE sau khi nha");
    EXPECT(s.btn().isIdle(), "isIdle() phai true de manager duoc ngu");
}

static void test_double_click_hold_fires_once()
{
    // Giu 3 giay = 300 chu ky poll deu thoa nguong. Neu thieu trang thai
    // WAIT_RELEASE_HOLD thi su kien se ban ra moi chu ky.
    Sim s(holdCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 120);
    s.feed(true, 3000);
    s.feed(false, 200);
    int count = 0;
    for (auto e : s.events) {
        if (e == ButtonEvent::DOUBLE_CLICK_HOLD) ++count;
    }
    EXPECT(count == 1, "DOUBLE_CLICK_HOLD phai ban DUNG mot lan, thuc te %d", count);
}

static void test_double_click_hold_timed_from_second_press()
{
    // Day la ca chong hoi quy quan trong nhat cua tinh nang nay.
    // canh nhan 1 tai t=10, canh nhan 2 tai t=190, longPressMs=800.
    //   dung  : do tu cu nhan THU HAI  -> 190 + 800 = 990
    //   sai   : do tu pressTimestamp_  ->  10 + 800 = 810
    // Do tu moc dau tien se lam nguong ngan di dung bang khoang nghi giua hai click.
    Sim s(holdCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 120);
    s.feed(true, 2000);
    s.feed(false, 200);

    EXPECT(s.stamps.size() == 2, "mong 2 su kien, nhan: %s", join(s.events).c_str());
    if (s.stamps.size() == 2) {
        EXPECT(s.stamps[0] == 190, "DOUBLE_CLICK tai t=%u, mong 190", s.stamps[0]);
        EXPECT(s.stamps[1] == 990,
               "DOUBLE_CLICK_HOLD tai t=%u, mong 990 (=190+800). Neu ra 810 tuc la "
               "dang do nham tu pressTimestamp_", s.stamps[1]);
    }
}

static void test_double_click_hold_keeps_gesture_start()
{
    // pressTimestampMs van tro ve dau cu chi, tuc cu nhan THU NHAT.
    Sim s(holdCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 120);
    s.feed(true, 2000);
    s.feed(false, 200);
    EXPECT(s.pressStamps.size() == 2, "mong 2 su kien");
    if (s.pressStamps.size() == 2) {
        EXPECT(s.pressStamps[0] == 10 && s.pressStamps[1] == 10,
               "ca hai su kien cung mot cu chi -> pressTimestampMs = 10, thuc te %u va %u",
               s.pressStamps[0], s.pressStamps[1]);
    }
}

static void test_double_click_hold_released_early()
{
    // Nha tay truoc nguong -> chi co DOUBLE_CLICK, khong co HOLD.
    Sim s(holdCfg(), 10);
    s.feed(true, 60);
    s.feed(false, 120);
    s.feed(true, 400);       // < longPressMs 800
    s.feed(false, 300);
    EXPECT(seq(s.events, {ButtonEvent::DOUBLE_CLICK}),
           "nha som thi khong duoc co DOUBLE_CLICK_HOLD. nhan: %s",
           join(s.events).c_str());
}

static void test_plain_long_press_unaffected()
{
    // Bat co moi khong duoc dung toi duong long press thong thuong.
    Sim s(holdCfg(), 10);
    s.feed(true, 1000);
    s.feed(false, 200);
    EXPECT(seq(s.events, {ButtonEvent::LONG_PRESS, ButtonEvent::LONG_PRESS_RELEASE}),
           "long press thuong phai giu nguyen. nhan: %s", join(s.events).c_str());
}

// ------------------------------------------------------------------------ main

int main()
{
    struct { const char* name; void (*fn)(); } tests[] = {
        {"nhan ngan -> mot CLICK",                 test_single_click},
        {"nhan hai lan -> DOUBLE_CLICK",           test_double_click},
        {"giu lau -> LONG_PRESS + RELEASE",        test_long_press},
        {"nhieu lien tuc -> khong su kien",        test_chatter_rejected},
        {"tat double click -> CLICK ngay",         test_double_click_disabled},
        {"nowMs gan diem tran uint32",             test_uint32_wrap},
        {"long press vat qua diem tran",           test_long_press_wrap},
        {"double click roi giu 3 giay",            test_double_click_then_hold},
        {"canh thang timeout cung chu ky",         test_edge_beats_timeout},
        {"cham mot chu ky -> hai CLICK",           test_edge_one_cycle_late},
        {"pressTimestampMs",                       test_press_timestamp},
        {"debounceCountFor()",                     test_debounce_count_for},
        {"doc lap tick rate",                      test_tick_rate_independence},
        {"debounceMaxCnt = 0 bi ep ve 1",          test_debounce_zero_forced_to_one},
        {"PRESS_DOWN mac dinh tat",                test_press_down_off_by_default},
        {"PRESS_DOWN bat -> PRESS_DOWN + CLICK",   test_press_down_emitted},
        {"PRESS_DOWN phat ngay, khong cho",        test_press_down_is_immediate},
        {"double click chi mot PRESS_DOWN",        test_press_down_once_per_double_click},
        {"PRESS_DOWN + LONG_PRESS + RELEASE",      test_press_down_long_press},
        {"PRESS_DOWN van bi debounce",             test_press_down_still_debounced},
        {"reset()",                                test_reset},
        {"bat hold -> DOUBLE_CLICK_HOLD",          test_double_click_hold_enabled},
        {"DOUBLE_CLICK_HOLD chi ban mot lan",      test_double_click_hold_fires_once},
        {"hold do tu cu nhan THU HAI",             test_double_click_hold_timed_from_second_press},
        {"hold giu nguyen moc dau cu chi",         test_double_click_hold_keeps_gesture_start},
        {"nha som -> khong co hold",               test_double_click_hold_released_early},
        {"long press thuong khong bi anh huong",   test_plain_long_press_unaffected},
    };

    std::printf("=== test Button (logic thuan) ===\n");
    for (auto& t : tests) {
        int before = g_failed;
        t.fn();
        std::printf("  [%s] %s\n", g_failed == before ? "OK  " : "FAIL", t.name);
    }
    std::printf("--- %d kiem tra, %d that bai ---\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
