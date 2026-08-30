// Tests for the sleep/wake sequencer. Run on a PC with g++.
//   ./run_tests.sh
//
// This is the reason components/display has a pure layer at all: every rule of
// display.md section 11.2 is checked here, with no board and no eyeballs.
#include "display.hpp"

#include <cstdint>
#include <cstdio>
#include <vector>

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

static const char* name(Step s)
{
    switch (s) {
    case Step::NONE:                return "NONE";
    case Step::BACKLIGHT_OFF: return "BACKLIGHT_OFF";
    case Step::APPLY_BRIGHTNESS:   return "APPLY_BRIGHTNESS";
    case Step::WAIT_TRANSFER:       return "WAIT_TRANSFER";
    case Step::PANEL_OFF:           return "PANEL_OFF";
    case Step::PANEL_ON:            return "PANEL_ON";
    case Step::PANEL_SLEEP_IN:      return "PANEL_SLEEP_IN";
    case Step::PANEL_SLEEP_OUT:     return "PANEL_SLEEP_OUT";
    case Step::REQUEST_REDRAW:      return "REQUEST_REDRAW";
    }
    return "?";
}

static const char* name(State s)
{
    switch (s) {
    case State::UNINITIALIZED: return "UNINITIALIZED";
    case State::AWAKE:         return "AWAKE";
    case State::DIMMED:        return "DIMMED";
    case State::PANEL_SLEEP:   return "PANEL_SLEEP";
    }
    return "?";
}

// Drive the policy to completion, the way the UI task would.
static std::vector<Step> drain(PowerPolicy& p, int limit = 16)
{
    std::vector<Step> steps;
    while (limit-- > 0) {
        const Step s = p.nextStep();
        if (s == Step::NONE) {
            break;
        }
        steps.push_back(s);
        p.stepDone();
    }
    return steps;
}

static bool matches(const std::vector<Step>& got, const std::vector<Step>& want)
{
    return got == want;
}

static void dump(const char* label, const std::vector<Step>& steps)
{
    std::printf("        %s:", label);
    for (Step s : steps) {
        std::printf(" %s", name(s));
    }
    std::printf("\n");
}

// init() leaves the panel on but the backlight at duty 0, so reaching AWAKE
// takes the redraw and the fade. Tests that want a lit screen start here.
static PowerPolicy awakePolicy()
{
    PowerPolicy p;
    p.onInitialized(80);
    drain(p);
    return p;
}

// -------------------------------------------------------------------- startup

static void test_startup()
{
    std::printf("startup\n");

    PowerPolicy p;
    EXPECT(p.state() == State::UNINITIALIZED, "starts uninitialized");
    EXPECT(!p.acceptsFlush(), "no flush before init");
    EXPECT(p.nextStep() == Step::NONE, "no work before init");

    // Requests before init must not arm anything.
    p.request(Request::SLEEP);
    EXPECT(p.state() == State::UNINITIALIZED, "requests ignored before init");
    EXPECT(p.nextStep() == Step::NONE, "still no work before init");

    // THE REGRESSION TEST FOR THE DARK-SCREEN BUG.
    //
    // init() ends with the panel on (disp_on_off) but the backlight still at
    // duty 0, because LEDC is configured first specifically to hold GPIO15 low.
    // onInitialized() must record THAT, not the intention that the display is
    // "up". A version that set backlightUp_ = true left the policy believing the
    // light was already on, so it never emitted APPLY_BRIGHTNESS and the panel
    // stayed dark until an unrelated setBrightness() happened to change the
    // value seconds later.
    p.onInitialized(80);
    EXPECT(p.brightness() == 80, "brightness recorded");
    EXPECT(p.target() == State::AWAKE, "init aims for AWAKE");
    EXPECT(p.state() == State::DIMMED,
           "the backlight is OFF right after init, so the state is DIMMED, got %s",
           name(p.state()));
    EXPECT(p.busy(), "init owes a fade -- a settled policy here means a dark screen");
    EXPECT(p.acceptsFlush(), "flushes are accepted before the light comes up");

    // GRAM is undefined at power-on exactly as it is after SLPOUT, so the
    // redraw is owed too, and it must come first.
    const std::vector<Step> got  = drain(p);
    const std::vector<Step> want = {Step::REQUEST_REDRAW, Step::APPLY_BRIGHTNESS};
    EXPECT(matches(got, want), "init sequence wrong");
    if (!matches(got, want)) {
        dump("got ", got);
        dump("want", want);
    }

    EXPECT(p.state() == State::AWAKE, "settles in AWAKE, got %s", name(p.state()));
    EXPECT(!p.busy(), "and then there is nothing left to do");
}

// ------------------------------------------------------------ sleep sequence

static void test_sleep_sequence()
{
    std::printf("sleep sequence\n");

    PowerPolicy p = awakePolicy();
    p.request(Request::SLEEP);

    const std::vector<Step> got  = drain(p);
    const std::vector<Step> want = {
        Step::BACKLIGHT_OFF,
        Step::WAIT_TRANSFER,
        Step::PANEL_OFF,
        Step::PANEL_SLEEP_IN,
    };

    EXPECT(matches(got, want), "sleep order wrong");
    if (!matches(got, want)) {
        dump("got ", got);
        dump("want", want);
    }

    EXPECT(p.state() == State::PANEL_SLEEP, "ends asleep, got %s", name(p.state()));
    EXPECT(!p.acceptsFlush(), "PANEL_SLEEP rejects flush");
    EXPECT(!p.busy(), "sequence complete");

    // The light goes out before the wait, never after: a torn last frame must
    // not be visible.
    EXPECT(got.front() == Step::BACKLIGHT_OFF, "backlight goes first");

    // Sleeping from DIMMED skips the fade -- the light is already out.
    PowerPolicy q = awakePolicy();
    q.request(Request::DIM);
    drain(q);
    q.request(Request::SLEEP);
    const std::vector<Step> fromDim = drain(q);
    EXPECT(matches(fromDim, {Step::WAIT_TRANSFER, Step::PANEL_OFF, Step::PANEL_SLEEP_IN}),
           "sleeping from DIMMED does not re-fade an already dark backlight");
}

// ------------------------------------------------------------- wake sequence

static void test_wake_sequence()
{
    std::printf("wake sequence\n");

    PowerPolicy p = awakePolicy();
    p.request(Request::SLEEP);
    drain(p);

    p.request(Request::WAKE);
    const std::vector<Step> got  = drain(p);
    const std::vector<Step> want = {
        Step::PANEL_SLEEP_OUT,
        Step::PANEL_ON,
        Step::REQUEST_REDRAW,
        Step::APPLY_BRIGHTNESS,
    };

    EXPECT(matches(got, want), "wake order wrong");
    if (!matches(got, want)) {
        dump("got ", got);
        dump("want", want);
    }

    // Rule 4 of section 11.2. GRAM contents are undefined across SLPIN/SLPOUT,
    // so lighting up before the redraw shows the user a frame of garbage.
    bool redrawBeforeLight = false;
    for (Step s : got) {
        if (s == Step::REQUEST_REDRAW) {
            redrawBeforeLight = true;
        }
        if (s == Step::APPLY_BRIGHTNESS) {
            break;
        }
    }
    EXPECT(redrawBeforeLight, "REQUEST_REDRAW must precede APPLY_BRIGHTNESS");

    EXPECT(p.state() == State::AWAKE, "ends awake, got %s", name(p.state()));
    EXPECT(p.acceptsFlush(), "AWAKE accepts flush again");
}

// ------------------------------------------------------------ DIMMED is live

static void test_dimmed_still_draws()
{
    std::printf("DIMMED\n");

    PowerPolicy p = awakePolicy();
    p.request(Request::DIM);

    const std::vector<Step> got = drain(p);
    EXPECT(matches(got, {Step::BACKLIGHT_OFF}), "dimming is one step");
    EXPECT(p.state() == State::DIMMED, "state is DIMMED, got %s", name(p.state()));

    // The whole reason DIMMED exists: the panel stays live, so re-lighting is
    // instant instead of costing the 100 ms of SLPOUT (section 11.3).
    EXPECT(p.acceptsFlush(), "DIMMED still accepts flushes");

    p.request(Request::WAKE);
    const std::vector<Step> back = drain(p);
    EXPECT(matches(back, {Step::APPLY_BRIGHTNESS}),
           "leaving DIMMED is one step -- no panel commands, no redraw");
    EXPECT(p.state() == State::AWAKE, "back to AWAKE");
}

// --------------------------------------------------------- the flush race

static void test_flush_blocked_immediately()
{
    std::printf("flush gating\n");

    PowerPolicy p = awakePolicy();
    EXPECT(p.acceptsFlush(), "flushes accepted while awake");

    // Rule 2 of section 11.2, and the sharpest rule in the class. The flush
    // gate must shut the instant sleep is REQUESTED, not once PANEL_OFF has
    // been executed -- otherwise LVGL can start a transfer in the window
    // between WAIT_TRANSFER and PANEL_OFF, which is the exact case
    // WAIT_TRANSFER exists to rule out.
    p.request(Request::SLEEP);
    EXPECT(!p.acceptsFlush(),
           "flushes must be refused immediately on request, before any step runs");

    // ... and stay refused through every intermediate step.
    while (p.nextStep() != Step::NONE) {
        p.stepDone();
        EXPECT(!p.acceptsFlush(), "still refused mid-sequence");
    }
}

// ------------------------------------------------------------ last state wins

static void test_last_state_wins()
{
    std::printf("last state wins\n");

    // A wrist raised and lowered twice in a second is ordinary. A queue of
    // steps would stack up stale commands and blink the screen; deriving each
    // step from what has actually been applied cannot.
    PowerPolicy p = awakePolicy();
    p.request(Request::SLEEP);

    // Get two steps into the sleep: backlight is out, transfers quiesced.
    p.stepDone();  // BACKLIGHT_OFF
    p.stepDone();  // WAIT_TRANSFER
    EXPECT(p.nextStep() == Step::PANEL_OFF, "mid-sleep, next would be PANEL_OFF");

    // Now reverse. The panel was never turned off, so waking must NOT emit
    // PANEL_SLEEP_OUT or PANEL_ON -- there is nothing to undo.
    p.request(Request::WAKE);
    const std::vector<Step> got = drain(p);
    EXPECT(matches(got, {Step::APPLY_BRIGHTNESS}),
           "reversing mid-sleep only re-lights; the panel was never switched off");
    if (!matches(got, {Step::APPLY_BRIGHTNESS})) {
        dump("got", got);
    }
    EXPECT(p.state() == State::AWAKE, "ends awake, got %s", name(p.state()));
    EXPECT(p.acceptsFlush(), "flushes allowed again");

    // Reversing the other way, deep into a wake.
    PowerPolicy q = awakePolicy();
    q.request(Request::SLEEP);
    drain(q);
    q.request(Request::WAKE);
    q.stepDone();  // PANEL_SLEEP_OUT
    q.stepDone();  // PANEL_ON
    q.request(Request::SLEEP);
    const std::vector<Step> reversed = drain(q);
    // Backlight never came up, so no fade down is needed.
    EXPECT(matches(reversed, {Step::WAIT_TRANSFER, Step::PANEL_OFF, Step::PANEL_SLEEP_IN}),
           "reversing mid-wake undoes only what was applied");
    if (!matches(reversed, {Step::WAIT_TRANSFER, Step::PANEL_OFF, Step::PANEL_SLEEP_IN})) {
        dump("got", reversed);
    }

    // Requests do not queue: three in a row leave only the last one standing.
    PowerPolicy r = awakePolicy();
    r.request(Request::SLEEP);
    r.request(Request::WAKE);
    r.request(Request::SLEEP);
    r.request(Request::WAKE);
    EXPECT(r.target() == State::AWAKE, "only the final request survives");
    EXPECT(!r.busy(), "already awake, so nothing to do at all");
}

// -------------------------------------------------------- WAIT_TRANSFER fails

static void test_wait_transfer_timeout_advances()
{
    std::printf("WAIT_TRANSFER timeout\n");

    // Section 7.3: a timed-out wait still advances. Leaving the screen lit
    // forever is worse than sleeping with a transfer possibly in flight, and
    // the UI task must not be trapped in the sequence.
    PowerPolicy p = awakePolicy();
    p.request(Request::SLEEP);

    p.stepDone();  // BACKLIGHT_OFF
    EXPECT(p.nextStep() == Step::WAIT_TRANSFER, "wait comes after the fade");

    // The manager reports ESP_ERR_TIMEOUT but calls stepDone() regardless.
    p.stepDone();

    EXPECT(p.nextStep() == Step::PANEL_OFF, "a failed wait does not stall the sequence");
    drain(p);
    EXPECT(p.state() == State::PANEL_SLEEP, "the panel still reaches sleep");
}

// ------------------------------------------------------------- brightness

static void test_brightness()
{
    std::printf("brightness\n");

    PowerPolicy p = awakePolicy();
    EXPECT(!p.busy(), "settled");

    // Changing brightness while lit needs one step to apply it.
    p.request(Request::SET_BRIGHTNESS, 30);
    EXPECT(p.brightness() == 30, "value stored");
    EXPECT(p.nextStep() == Step::APPLY_BRIGHTNESS, "a lit backlight is re-applied");
    drain(p);
    EXPECT(p.state() == State::AWAKE, "still awake after a brightness change");

    // Setting the same value again is not work.
    p.request(Request::SET_BRIGHTNESS, 30);
    EXPECT(!p.busy(), "an unchanged brightness emits no step");

    // While dimmed there is nothing to apply; the value is picked up by the
    // next fade up.
    p.request(Request::DIM);
    drain(p);
    p.request(Request::SET_BRIGHTNESS, 55);
    EXPECT(p.brightness() == 55, "value stored while dark");
    EXPECT(!p.busy(), "no step needed while the backlight is off");
    EXPECT(p.state() == State::DIMMED, "still dimmed");

    p.request(Request::WAKE);
    EXPECT(p.nextStep() == Step::APPLY_BRIGHTNESS, "waking applies the new value");
    drain(p);

    p.request(Request::SET_BRIGHTNESS, 200);
    EXPECT(p.brightness() == 100, "over 100 clamps, got %u", p.brightness());
}

// ------------------------------------------------------------ DIM while asleep

static void test_dim_while_asleep()
{
    std::printf("DIM while asleep\n");

    PowerPolicy p = awakePolicy();
    p.request(Request::SLEEP);
    drain(p);

    // Waking the panel just to dim it would cost over 100 ms for a screen
    // nobody is looking at.
    p.request(Request::DIM);
    EXPECT(p.target() == State::PANEL_SLEEP, "DIM does not wake a sleeping panel");
    EXPECT(!p.busy(), "no work generated");
    EXPECT(p.state() == State::PANEL_SLEEP, "still asleep");
    EXPECT(!p.acceptsFlush(), "still refusing flushes");
}

// ------------------------------------------------------------------ stability

static void test_repeated_cycles()
{
    std::printf("repeated cycles\n");

    // The board test runs 500 of these; here we only prove the state machine
    // does not drift, which is the part that does not need hardware.
    PowerPolicy p = awakePolicy();

    for (int i = 0; i < 500; ++i) {
        p.request(Request::SLEEP);
        drain(p);
        if (p.state() != State::PANEL_SLEEP) {
            EXPECT(false, "cycle %d did not reach PANEL_SLEEP", i);
            break;
        }
        p.request(Request::WAKE);
        drain(p);
        if (p.state() != State::AWAKE) {
            EXPECT(false, "cycle %d did not return to AWAKE", i);
            break;
        }
    }

    EXPECT(p.state() == State::AWAKE, "ends awake after 500 cycles");
    EXPECT(p.acceptsFlush(), "and still drawable");
    EXPECT(!p.busy(), "and settled");
}

int main()
{
    std::printf("== display power policy ==\n");

    test_startup();
    test_sleep_sequence();
    test_wake_sequence();
    test_dimmed_still_draws();
    test_flush_blocked_immediately();
    test_last_state_wins();
    test_wait_transfer_timeout_advances();
    test_brightness();
    test_dim_while_asleep();
    test_repeated_cycles();

    std::printf("%d checks, %d failed\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
