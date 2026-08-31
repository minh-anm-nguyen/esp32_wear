// Tests for the navigation policy.
//   ./run_tests.sh
//
// The bugs these prevent are all "the watch did something the user did not ask
// for": a double tap rebuilding an app, Back leaving the launcher for nowhere,
// a deep stack quietly forgetting where home was.
#include "nav_stack.hpp"

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

static void testStartsAtLauncher()
{
    NavStack<4> n;
    EXPECT(n.atLauncher(), "khoi dau phai o launcher");
    EXPECT(n.current() == kLauncherIndex, "current()==-1");
    EXPECT(n.depth() == 0, "depth 0");
}

static void testPushAndPop()
{
    NavStack<4> n;
    EXPECT(n.push(2), "mo app 2");
    EXPECT(n.current() == 2, "dang o app 2, thuc te %d", n.current());
    EXPECT(!n.atLauncher(), "khong con o launcher");

    EXPECT(n.pop(), "back duoc");
    EXPECT(n.atLauncher(), "back ve launcher");
}

static void testBackFromLauncherDoesNothing()
{
    // On a watch there is nothing behind the launcher. pop() must say so rather
    // than underflow into a negative depth.
    NavStack<4> n;
    EXPECT(!n.pop(), "back o launcher phai tra false");
    EXPECT(n.atLauncher(), "van o launcher");
    EXPECT(n.depth() == 0, "depth khong duoc am");
}

static void testDoubleTapIsRejected()
{
    // THE test of this file. One double tap on an icon posts two SHOW_APP
    // commands. Obeying both would tear the app down and rebuild it.
    NavStack<4> n;
    EXPECT(n.push(1), "lan mo dau tien");
    EXPECT(!n.push(1), "mo LAI cung app phai bi tu choi");
    EXPECT(n.depth() == 1, "khong duoc chong len, depth=%u", n.depth());
    EXPECT(n.rejectedRepeat() == 1, "phai dem lan tu choi");

    // A different app from inside one is still legal.
    EXPECT(n.push(2), "mo app khac thi duoc");
    EXPECT(n.depth() == 2, "depth 2");
}

static void testFullStackRefusesRatherThanDropsHome()
{
    NavStack<2> n;
    EXPECT(n.push(0), "1");
    EXPECT(n.push(1), "2");
    EXPECT(!n.push(2), "day roi -> tu choi");
    EXPECT(n.rejectedFull() == 1, "dem lan day");

    // Refusing keeps the path home intact. Dropping the bottom entry instead
    // would make Back arrive somewhere the user was never at.
    EXPECT(n.current() == 1, "van o app 1");
    EXPECT(n.pop() && n.current() == 0, "back ve app 0");
    EXPECT(n.pop() && n.atLauncher(), "back ve launcher");
}

static void testPushLauncherIsRejected()
{
    // "Go to the launcher" is reset(), not push(-1). Allowing the latter would
    // put a launcher entry ON the stack and make Back cycle through launchers.
    NavStack<4> n;
    EXPECT(!n.push(kLauncherIndex), "push(-1) phai bi tu choi");
    EXPECT(!n.push(-7), "index am khac cung bi tu choi");
    EXPECT(n.depth() == 0, "khong co gi vao stack");
}

static void testResetGoesHomeFromAnyDepth()
{
    NavStack<4> n;
    n.push(0);
    n.push(1);
    n.push(2);
    EXPECT(n.depth() == 3, "sau ba lan mo");

    n.reset();
    EXPECT(n.atLauncher(), "reset ve thang launcher");
    EXPECT(n.depth() == 0, "depth 0");

    // And the stack is genuinely usable again, not just reporting zero.
    EXPECT(n.push(0), "mo lai duoc sau reset");
    EXPECT(n.current() == 0, "dung app");
}

static void testReopenAfterBackIsAllowed()
{
    // The repeat guard must be about the CURRENT screen, not history: leaving
    // an app and coming straight back to it is an ordinary thing to do.
    NavStack<4> n;
    n.push(1);
    n.pop();
    EXPECT(n.push(1), "mo lai app 1 sau khi back phai duoc");
    EXPECT(n.current() == 1, "dang o app 1");
}

int main()
{
    std::printf("nav_stack\n");

    testStartsAtLauncher();
    testPushAndPop();
    testBackFromLauncherDoesNothing();
    testDoubleTapIsRejected();
    testFullStackRefusesRatherThanDropsHome();
    testPushLauncherIsRejected();
    testResetGoesHomeFromAnyDepth();
    testReopenAfterBackIsAllowed();

    std::printf("  %d kiem tra, %d that bai\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
