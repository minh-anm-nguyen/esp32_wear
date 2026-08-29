// Tests for the pure logic layer, run on a PC with g++. No chip, no ESP-IDF.
//   ./run_tests.sh
#include "buzzer.hpp"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace buzzer;

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

// One hardware write, as the driver layer would see it.
struct Step {
    uint32_t t;
    bool     on;
    uint16_t freqHz;
    uint8_t  volume;

    bool operator==(const Step& o) const
    {
        return t == o.t && on == o.on && freqHz == o.freqHz && volume == o.volume;
    }
};

static std::string join(const std::vector<Step>& v)
{
    if (v.empty()) return "(rong)";
    std::string s;
    char        buf[64];
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) s += " , ";
        if (v[i].on) {
            std::snprintf(buf, sizeof(buf), "%u:ON(%uHz,%u%%)", v[i].t,
                          static_cast<unsigned>(v[i].freqHz),
                          static_cast<unsigned>(v[i].volume));
        } else {
            std::snprintf(buf, sizeof(buf), "%u:OFF", v[i].t);
        }
        s += buf;
    }
    return s;
}

// Drives a Buzzer the way BuzzerManager::run() does: it never polls blindly, it
// sleeps exactly as long as nextDelayMs() asks for.
class Sim {
public:
    explicit Sim(const BuzzerSpec& cfg, uint32_t startMs = 0)
        : bz_(cfg), t_(startMs)
    {
    }

    void pump()
    {
        const ToneOutput o = bz_.update(t_);
        if (o.changed) {
            log.push_back(Step{t_, o.on, o.freqHz, o.volume});
        }
    }

    void advance(uint32_t durationMs)
    {
        int guard = 200000;  // a stuck sequencer must fail the test, not hang it
        pump();

        uint32_t left = durationMs;
        while (left > 0 && --guard > 0) {
            uint32_t d = bz_.nextDelayMs(t_);
            if (d == kSleepForever || d > left) {
                d = left;
            }
            if (d == 0) {
                pump();  // overdue: act now, do not consume time
                continue;
            }
            t_   += d;
            left -= d;
            pump();
        }
        EXPECT(guard > 0, "advance() khong tien trien: sequencer bi ket");
    }

    Buzzer&  bz()  { return bz_; }
    uint32_t now() const { return t_; }

    std::vector<Step> log;

private:
    Buzzer   bz_;
    uint32_t t_;
};

static BuzzerSpec passiveCfg()
{
    BuzzerSpec c{};
    c.type      = BuzzerType::PASSIVE;
    c.maxVolume = 100;
    c.minFreqHz = 100;
    c.maxFreqHz = 10000;
    return c;
}

static BuzzerSpec activeCfg()
{
    BuzzerSpec c = passiveCfg();
    c.type         = BuzzerType::ACTIVE;
    return c;
}

static bool seq(const std::vector<Step>& got, const std::vector<Step>& want)
{
    return got == want;
}

static size_t countOn(const std::vector<Step>& v)
{
    size_t n = 0;
    for (const auto& s : v) {
        if (s.on) ++n;
    }
    return n;
}

// ------------------------------------------------------------------ test cases

static const Note kOneBeep[] = {{2700, 30, 60}};

static void test_single_beep()
{
    Sim s(passiveCfg());
    s.bz().play(makePattern(kOneBeep), s.now());
    s.advance(200);

    EXPECT(seq(s.log, {{0, true, 2700, 60}, {30, false, 0, 0}}),
           "mong ON@0 roi OFF@30, nhan: %s", join(s.log).c_str());
    EXPECT(s.bz().isIdle(), "het pattern phai ve idle");
}

// A rest is a Note with freqHz == 0: silent, but it still burns its duration.
static const Note kBeepGapBeep[] = {{2700, 50, 80}, {0, 40, 0}, {2700, 50, 80}};

static void test_rest_is_silent_but_costs_time()
{
    Sim s(passiveCfg());
    s.bz().play(makePattern(kBeepGapBeep), s.now());
    s.advance(300);

    EXPECT(seq(s.log, {{0, true, 2700, 80},
                       {50, false, 0, 0},
                       {90, true, 2700, 80},
                       {140, false, 0, 0}}),
           "khoang lang phai im nhung van an du 40ms, nhan: %s", join(s.log).c_str());
}

static const Note kTick[] = {{2700, 10, 100}, {0, 10, 0}};

static void test_repeat_three_times()
{
    Sim s(passiveCfg());
    s.bz().play(makePattern(kTick, /*repeat=*/3), s.now());
    s.advance(500);

    EXPECT(countOn(s.log) == 3, "repeat=3 phai keu dung 3 lan, nhan: %s",
           join(s.log).c_str());
    EXPECT(s.bz().isIdle(), "sau 3 vong phai ve idle");
}

static void test_repeat_zero_loops_until_stop()
{
    Sim s(passiveCfg());
    s.bz().play(makePattern(kTick, /*repeat=*/0), s.now());
    s.advance(1000);

    EXPECT(!s.bz().isIdle(), "repeat=0 khong duoc tu dung");
    EXPECT(countOn(s.log) > 40, "1000ms / 20ms mot vong phai keu nhieu lan, nhan %zu",
           countOn(s.log));

    const size_t before = s.log.size();
    s.bz().stop();
    s.pump();
    EXPECT(s.log.size() == before + 1 && !s.log.back().on,
           "stop() phai phat dung mot lenh TAT, nhan: %s", join(s.log).c_str());
    EXPECT(s.bz().isIdle(), "sau stop() phai idle");
}

// The trap this test exists for: after stop() the state is IDLE while the OFF
// command has not been applied. A nextDelayMs() that only looked at the state
// would answer kSleepForever and leave the piezo screaming.
static void test_pending_output_blocks_sleep()
{
    Sim s(passiveCfg());
    s.bz().play(makePattern(kTick, 0), s.now());
    s.pump();
    s.bz().stop();

    EXPECT(s.bz().getState() == BuzzerState::IDLE, "stop() phai dat state IDLE");
    EXPECT(!s.bz().isIdle(), "con lenh TAT chua ap thi chua duoc coi la idle");
    EXPECT(s.bz().nextDelayMs(s.now()) == 0,
           "con lenh chua ap thi nextDelayMs phai la 0, nhan %u",
           s.bz().nextDelayMs(s.now()));

    s.pump();
    EXPECT(s.bz().isIdle(), "ap xong lenh TAT thi moi idle");
    EXPECT(s.bz().nextDelayMs(s.now()) == kSleepForever,
           "idle phai ngu vo han");
}

static const Note kLoud[] = {{2700, 500, 100}};
static const Note kQuiet[] = {{1000, 500, 50}};

static void test_priority_lower_refused()
{
    Sim s(passiveCfg());
    EXPECT(s.bz().play(makePattern(kLoud, 1, /*priority=*/200), s.now()),
           "pattern dau tien phai duoc nhan");
    s.advance(100);

    const size_t before = s.log.size();
    EXPECT(!s.bz().play(makePattern(kQuiet, 1, /*priority=*/10), s.now()),
           "uu tien thap hon phai bi tu choi");
    s.advance(100);
    EXPECT(s.log.size() == before, "bi tu choi thi khong duoc ghi de gi, nhan: %s",
           join(s.log).c_str());
}

static void test_priority_higher_preempts()
{
    Sim s(passiveCfg());
    s.bz().play(makePattern(kQuiet, 1, /*priority=*/10), s.now());
    s.advance(100);

    EXPECT(s.bz().play(makePattern(kLoud, 1, /*priority=*/200), s.now()),
           "uu tien cao hon phai chen duoc");
    s.pump();
    EXPECT(!s.log.empty() && s.log.back().on && s.log.back().freqHz == 2700,
           "phai chuyen sang not cua pattern uu tien cao, nhan: %s",
           join(s.log).c_str());
}

static void test_equal_priority_restarts()
{
    Sim s(passiveCfg());
    s.bz().play(makePattern(kOneBeep, 1, /*priority=*/10), s.now());
    s.advance(10);

    EXPECT(s.bz().play(makePattern(kOneBeep, 1, /*priority=*/10), s.now()),
           "uu tien bang nhau phai thay the, khong bi nuot");
    // Restarted at t=10, so the beep now ends at 40 instead of 30.
    s.advance(100);
    EXPECT(!s.log.empty() && !s.log.back().on && s.log.back().t == 40,
           "phat lai tu dau thi phai ket thuc o t=40, nhan: %s", join(s.log).c_str());
}

static const Note kTooHigh[] = {{20000, 40, 100}};

static void test_frequency_out_of_range_is_silent()
{
    BuzzerSpec c = passiveCfg();
    c.maxFreqHz    = 10000;
    Sim s(c);
    s.bz().play(makePattern(kTooHigh), s.now());
    s.advance(200);

    // Silent, yet it still consumed its 40 ms: one OFF at 0, one OFF at 40.
    EXPECT(seq(s.log, {{0, false, 0, 0}, {40, false, 0, 0}}),
           "tan so ngoai dai phai im nhung van an du 40ms, nhan: %s",
           join(s.log).c_str());
}

static const Note kMotor[] = {{kOn, 40, 100}};

static void test_active_ignores_frequency_range()
{
    // kOn == 1 is far below minFreqHz, which would silence a passive device.
    Sim s(activeCfg());
    s.bz().play(makePattern(kMotor), s.now());
    s.advance(200);

    EXPECT(s.log.size() == 2 && s.log[0].on,
           "buzzer chu dong khong quan tam tan so, nhan: %s", join(s.log).c_str());
}

static const Note kFullVolume[] = {{2700, 40, 100}};

static void test_volume_clamped_by_max()
{
    BuzzerSpec c = passiveCfg();
    c.maxVolume    = 40;
    Sim s(c);
    s.bz().play(makePattern(kFullVolume), s.now());
    s.advance(100);

    EXPECT(!s.log.empty() && s.log[0].volume == 40,
           "volume phai bi chan boi maxVolume=40, nhan %u",
           s.log.empty() ? 0u : static_cast<unsigned>(s.log[0].volume));
}

static const Note kZeroVolume[] = {{2700, 40, 0}};

static void test_zero_volume_is_silent()
{
    Sim s(passiveCfg());
    s.bz().play(makePattern(kZeroVolume), s.now());
    s.advance(100);

    EXPECT(countOn(s.log) == 0, "volume 0 phai im, nhan: %s", join(s.log).c_str());
}

static const Note kZeroDuration[] = {{2700, 0, 100}, {2700, 20, 100}};

static void test_zero_duration_does_not_stall()
{
    Sim s(passiveCfg());
    s.bz().play(makePattern(kZeroDuration), s.now());
    s.advance(200);

    EXPECT(s.bz().isIdle(),
           "not dai 0ms phai bi ep len 1ms de vong lap tien duoc, nhan: %s",
           join(s.log).c_str());
}

static void test_uint32_wrap()
{
    // The pattern starts before the wrap point and finishes after it.
    Sim s(passiveCfg(), /*startMs=*/0xFFFFFFF0u);
    s.bz().play(makePattern(kBeepGapBeep), s.now());
    s.advance(300);

    EXPECT(countOn(s.log) == 2 && s.bz().isIdle(),
           "vat qua diem tran uint32 van phai keu du 2 lan roi idle, nhan: %s",
           join(s.log).c_str());
}

static void test_empty_pattern_refused()
{
    Sim s(passiveCfg());
    EXPECT(!s.bz().play(Pattern{}, s.now()), "pattern rong phai bi tu choi");
    EXPECT(s.bz().isIdle(), "bi tu choi thi van phai idle");
}

static void test_stop_when_idle_is_noop()
{
    Sim s(passiveCfg());
    s.bz().stop();
    s.pump();
    EXPECT(s.log.empty(), "stop() luc dang idle khong duoc ghi gi ra phan cung, nhan: %s",
           join(s.log).c_str());
}

static void test_reset()
{
    Sim s(passiveCfg());
    s.bz().play(makePattern(kTick, 0), s.now());
    s.advance(100);
    EXPECT(s.bz().getState() == BuzzerState::PLAYING, "phai dang PLAYING");

    s.bz().reset();
    EXPECT(s.bz().getState() == BuzzerState::IDLE, "reset phai ve IDLE");
    EXPECT(s.bz().isIdle(), "reset phai xoa ca co outputPending_");
    EXPECT(s.bz().nextDelayMs(s.now()) == kSleepForever, "reset phai ngu vo han");
}

static void test_stock_patterns_are_consistent()
{
    // makePattern() deduces the count, so this only guards the hand written
    // priorities: an alarm must outrank every UI sound.
    EXPECT(patterns::kAlarm.priority > patterns::kError.priority &&
               patterns::kError.priority > patterns::kOk.priority &&
               patterns::kOk.priority > patterns::kClick.priority,
           "thu tu uu tien cua cac pattern dung san bi sai");
    EXPECT(patterns::kAlarm.repeat == 0, "bao thuc phai lap vo han");
    EXPECT(patterns::kClick.count == 1, "kClick phai co dung 1 not");
}

// ------------------------------------------------------------------------ main

int main()
{
    struct { const char* name; void (*fn)(); } tests[] = {
        {"mot tieng bip -> ON roi OFF",            test_single_beep},
        {"not nghi im nhung van an thoi gian",     test_rest_is_silent_but_costs_time},
        {"repeat = 3 -> keu 3 lan",                test_repeat_three_times},
        {"repeat = 0 -> lap toi khi stop()",       test_repeat_zero_loops_until_stop},
        {"con lenh chua ap thi chua duoc ngu",     test_pending_output_blocks_sleep},
        {"uu tien thap hon bi tu choi",            test_priority_lower_refused},
        {"uu tien cao hon chen duoc",              test_priority_higher_preempts},
        {"uu tien bang nhau -> phat lai",          test_equal_priority_restarts},
        {"tan so ngoai dai -> im",                 test_frequency_out_of_range_is_silent},
        {"buzzer chu dong bo qua tan so",          test_active_ignores_frequency_range},
        {"volume bi chan boi maxVolume",           test_volume_clamped_by_max},
        {"volume 0 -> im",                         test_zero_volume_is_silent},
        {"not dai 0ms khong lam ket vong lap",     test_zero_duration_does_not_stall},
        {"nowMs gan diem tran uint32",             test_uint32_wrap},
        {"pattern rong bi tu choi",                test_empty_pattern_refused},
        {"stop() luc idle khong lam gi",           test_stop_when_idle_is_noop},
        {"reset()",                                test_reset},
        {"pattern dung san hop le",                test_stock_patterns_are_consistent},
    };

    std::printf("=== test Buzzer (logic thuan) ===\n");
    for (auto& t : tests) {
        int before = g_failed;
        t.fn();
        std::printf("  [%s] %s\n", g_failed == before ? "OK  " : "FAIL", t.name);
    }
    std::printf("--- %d kiem tra, %d that bai ---\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
