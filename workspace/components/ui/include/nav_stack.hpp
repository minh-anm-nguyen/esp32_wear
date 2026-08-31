// Where the user is, and how they get back. Pure: <cstdint> only, so the whole
// navigation policy is a host test.
//
// WHY THIS IS SEPARATE FROM AppHost
//
// AppHost does LVGL work -- creating screens, deleting object trees, measuring
// heap -- none of which can run on a PC. The DECISIONS, though, are exactly the
// part that bites: opening the same app twice from one double tap, going back
// from the launcher, running out of stack depth. Those are arithmetic, and
// arithmetic belongs in a test.
//
// See app.hpp rule 7: an app REQUESTS navigation, the framework DECIDES.
#pragma once

#include <cstdint>

namespace ui {

// The launcher is not an entry on the stack, it is the floor. An empty stack
// means "showing the launcher", which makes "can I go back?" the same question
// as "is the stack non-empty?" and removes a whole class of off-by-one.
inline constexpr int8_t kLauncherIndex = -1;

template <uint8_t Depth>
class NavStack {
    static_assert(Depth > 0, "a stack with no room can never leave the launcher");

public:
    int8_t  current() const { return depth_ == 0 ? kLauncherIndex : stack_[depth_ - 1]; }
    bool    atLauncher() const { return depth_ == 0; }
    uint8_t depth() const { return depth_; }
    uint8_t capacity() const { return Depth; }

    // Returns false when the request should be IGNORED rather than obeyed.
    //
    // Two reasons, and the first is the one that matters on a touch screen:
    //
    //   - already showing that app. A double tap on a launcher icon posts two
    //     SHOW_APP commands; obeying both tears down the app and rebuilds it,
    //     which looks like a flicker and throws away whatever the user had just
    //     started doing in it.
    //   - the stack is full. Refusing is better than silently dropping the
    //     bottom entry, which would make Back go somewhere the user never was.
    bool push(int8_t appIndex)
    {
        if (appIndex < 0) {
            return false;  // use reset() to go home; pushing "launcher" is a bug
        }
        if (current() == appIndex) {
            ++rejectedRepeat_;
            return false;
        }
        if (depth_ >= Depth) {
            ++rejectedFull_;
            return false;
        }
        stack_[depth_++] = appIndex;
        return true;
    }

    // False when there is nowhere to go: already at the launcher. The caller
    // decides what that means -- on a watch it is usually "do nothing", never
    // "exit", because there is nothing to exit to.
    bool pop()
    {
        if (depth_ == 0) {
            return false;
        }
        --depth_;
        return true;
    }

    // Straight home, however deep. What a long press or a HOME command does.
    void reset() { depth_ = 0; }

    uint32_t rejectedRepeat() const { return rejectedRepeat_; }
    uint32_t rejectedFull() const { return rejectedFull_; }

private:
    int8_t  stack_[Depth]{};
    uint8_t depth_{0};

    uint32_t rejectedRepeat_{0};
    uint32_t rejectedFull_{0};
};

}  // namespace ui
