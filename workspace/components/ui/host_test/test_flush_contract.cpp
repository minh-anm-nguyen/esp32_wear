// Tests for "exactly one lv_display_flush_ready() per flush".
//   ./run_tests.sh
//
// Every case here is a way the screen freezes or corrupts on hardware while the
// UI task stays alive and the log stays silent -- which is why they are worth
// catching on a PC instead.
#include "flush_coordinator.hpp"
#include "ui_command.hpp"

#include <cstdio>

using namespace ui;

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

// ------------------------------------------------------------- the happy path

static void testNormalFlushCompletesViaIsr()
{
    FlushCoordinator f;

    EXPECT(f.reserve(), "dat cho thanh cong -> phai cho ISR");
    EXPECT(f.pending(), "dang co transfer bay");

    EXPECT(f.completeFromIsr(), "ISR dau tien phai duoc chap nhan");
    EXPECT(!f.pending(), "xong thi khong con pending");
    EXPECT(f.completedByIsr() == 1, "dem dung");
    EXPECT(f.balanced(), "bat dau 1, ket thuc 1");
}

static void testManyFlushesStayBalanced()
{
    FlushCoordinator f;
    for (int i = 0; i < 100; ++i) {
        f.reserve();
        f.completeFromIsr();
    }
    EXPECT(f.started() == 100 && f.completedByIsr() == 100, "100 vao, 100 ra");
    EXPECT(f.balanced(), "can bang sau 100 flush");
}

// --------------------------------------------------- trap #16: the hang

static void testRejectedDrawMustBeCompletedByCaller()
{
    // drawRgb565() returns ESP_ERR_INVALID_STATE at PANEL_SLEEP. LVGL still
    // waits for flush_ready unconditionally, so the callback owes it one.
    //
    // Getting this wrong is the bug where the screen freezes after the FIRST
    // sleep, the task is alive, and nothing is logged.
    FlushCoordinator f;

    EXPECT(f.reserve(), "dat cho truoc khi ve");
    f.abandon();
    EXPECT(!f.pending(), "khong co gi dang bay");
    EXPECT(f.readyNowRejected() == 1, "dem duong bi tu choi");
    EXPECT(f.balanced(), "duong loi van phai can bang");
}

static void testSleepingDisplayNeverStalls()
{
    // A whole sleep period: every flush refused, every one still settled.
    FlushCoordinator f;
    for (int i = 0; i < 50; ++i) {
        EXPECT(f.reserve(), "dat cho lan %d", i);
        f.abandon();
    }
    EXPECT(f.readyNowRejected() == 50, "50 lan tu choi");
    EXPECT(f.balanced(), "khong lan nao bi treo");
}

// ------------------------------------------------ double / spurious completion

static void testSecondIsrForOneFlushIsRefused()
{
    FlushCoordinator f;
    f.reserve();

    EXPECT(f.completeFromIsr(), "ISR lan 1 duoc chap nhan");

    // The dangerous one. Reporting this to LVGL would release a buffer that
    // belongs to the NEXT flush.
    EXPECT(!f.completeFromIsr(), "ISR lan 2 phai BI TU CHOI");
    EXPECT(f.spurious() == 1, "phai dem la spurious");
    EXPECT(f.completedByIsr() == 1, "khong duoc dem thanh hai lan hoan tat");
    EXPECT(f.balanced(), "van can bang");
}

static void testIsrWithNothingPendingIsRefused()
{
    FlushCoordinator f;
    EXPECT(!f.completeFromIsr(), "ISR khi chua he flush -> tu choi");
    EXPECT(f.spurious() == 1, "dem spurious");
    EXPECT(f.balanced(), "khong lam lech so dem");
}

// ------------------------------------------------ trap: timeout reuses buffers

static void testTimeoutBlocksFurtherFlushes()
{
    FlushCoordinator f;
    f.reserve();
    EXPECT(f.pending(), "dang bay");

    f.onTimeout();

    EXPECT(!f.pending(), "het gio thi thoi cho");
    EXPECT(f.blocked(), "phai CHAN flush moi");
    EXPECT(f.timedOut() == 1, "dem timeout");

    // Why blocked matters: LVGL has already set flushing=0 and will render into
    // that buffer again, while the DMA may still be reading it. Queueing another
    // transfer on top is how a dead panel becomes memory corruption.
    EXPECT(!f.reserve(),
           "sau timeout, dat cho phai BI TU CHOI -- caller tu bao flush_ready");
    EXPECT(!f.pending(), "va khong duoc danh dau la dang bay");
    EXPECT(f.readyNowBlocked() == 1, "dem duong bi chan");
    EXPECT(f.balanced(), "van can bang");
}

static void testLateIsrAfterTimeoutIsRefused()
{
    // The completion that arrives after everyone gave up. It belongs to a
    // transfer nobody is waiting for any more.
    FlushCoordinator f;
    f.reserve();
    f.onTimeout();

    EXPECT(!f.completeFromIsr(), "ISR den muon sau timeout phai bi tu choi");
    EXPECT(f.spurious() == 1, "dem spurious");
    EXPECT(f.blocked(), "van con bi chan");
}

static void testRecoveryUnblocks()
{
    FlushCoordinator f;
    f.reserve();
    f.onTimeout();
    EXPECT(f.blocked(), "dang bi chan");

    f.resumeAfterRecovery();

    EXPECT(!f.blocked(), "sau khoi phuc phai nhan flush lai");
    EXPECT(f.recoveries() == 1, "dem lan khoi phuc");
    EXPECT(f.reserve(), "flush lai binh thuong");
    EXPECT(f.completeFromIsr(), "va hoan tat binh thuong");
}

// ------------------------------------------------------------ mixed sequences

static void testInterleavedSequenceStaysBalanced()
{
    FlushCoordinator f;

    f.reserve();  f.completeFromIsr();     // normal
    f.reserve();  f.abandon();             // panel asleep, draw refused
    f.reserve();  f.completeFromIsr();     // normal
    f.completeFromIsr();                   // spurious
    f.reserve();  f.onTimeout();           // dead
    f.reserve();                           // blocked -> refused
    f.resumeAfterRecovery();
    f.reserve();  f.completeFromIsr();     // normal again

    EXPECT(f.started() == 6, "6 lan reserve, thuc te %u", f.started());
    EXPECT(f.completedByIsr() == 3, "3 lan ISR hop le, thuc te %u",
           f.completedByIsr());
    EXPECT(f.readyNowRejected() == 1, "1 lan tu choi ve");
    EXPECT(f.readyNowBlocked() == 1, "1 lan bi chan");
    EXPECT(f.timedOut() == 1, "1 lan timeout");
    EXPECT(f.spurious() == 1, "1 lan spurious");
    EXPECT(f.balanced(), "chuoi tron lan van phai can bang");
}

// ------------------------------------------------------------- lane policy

static void testCriticalCommandsGetTheirOwnLane()
{
    EXPECT(laneOf(UiCommandType::WAKE) == Lane::Control, "WAKE la control");
    EXPECT(laneOf(UiCommandType::SET_BRIGHTNESS) == Lane::Control,
           "SET_BRIGHTNESS la control");
    EXPECT(laneOf(UiCommandType::SHOW_APP) == Lane::Control, "SHOW_APP la control");

    // These must not be able to queue behind a burst of navigation.
    EXPECT(laneOf(UiCommandType::ALARM_FIRED) == Lane::Critical,
           "ALARM_FIRED phai la critical");
    EXPECT(laneOf(UiCommandType::LOW_BATTERY) == Lane::Critical,
           "LOW_BATTERY phai la critical");
    EXPECT(laneOf(UiCommandType::PREPARE_SHUTDOWN) == Lane::Critical,
           "PREPARE_SHUTDOWN phai la critical");
}

static void testOnlyLiveStatesAcceptCommands()
{
    EXPECT(!acceptsCommands(RuntimeState::Uninitialized), "chua init -> tu choi");
    EXPECT(acceptsCommands(RuntimeState::Starting), "dang start -> nhan");
    EXPECT(acceptsCommands(RuntimeState::Running), "dang chay -> nhan");

    // The rule that stops a shutdown being undone by a queued WAKE.
    EXPECT(!acceptsCommands(RuntimeState::Stopping), "dang dung -> tu choi");
    EXPECT(!acceptsCommands(RuntimeState::Stopped), "da dung -> tu choi");
    EXPECT(!acceptsCommands(RuntimeState::Failed), "loi -> tu choi");
}

static void testCommandIsSmallAndTrivial()
{
    // The no-pointer rule, enforced by the compiler rather than by review.
    static_assert(sizeof(UiCommand) <= 8, "UiCommand phai nho: no duoc copy vao queue");
    EXPECT(sizeof(UiCommand) <= 8, "sizeof(UiCommand)=%zu", sizeof(UiCommand));
}

int main()
{
    std::printf("ui flush + command\n");

    testNormalFlushCompletesViaIsr();
    testManyFlushesStayBalanced();
    testRejectedDrawMustBeCompletedByCaller();
    testSleepingDisplayNeverStalls();
    testSecondIsrForOneFlushIsRefused();
    testIsrWithNothingPendingIsRefused();
    testTimeoutBlocksFurtherFlushes();
    testLateIsrAfterTimeoutIsRefused();
    testRecoveryUnblocks();
    testInterleavedSequenceStaysBalanced();
    testCriticalCommandsGetTheirOwnLane();
    testOnlyLiveStatesAcceptCommands();
    testCommandIsSmallAndTrivial();

    std::printf("  %d kiem tra, %d that bai\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
