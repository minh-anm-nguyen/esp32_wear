// One producer publishes "what is true now"; any number of readers ask "has it
// changed since I last looked". Pure logic -- includes only <cstdint> and
// <type_traits>, so the whole protocol is testable with a bare g++.
//
// WHY THIS EXISTS
//
// The first sketch of components/ui had ONE flat UiStateSnapshot carrying every
// value the UI might display, with dirty flags. That works for three screens
// and fails for fifteen apps, in five separate ways:
//
//   - every app #includes it, so adding a weather app rebuilds the alarm app;
//   - it becomes the union of everyone's needs, so nothing can ever be deleted;
//   - heart rate at 1 Hz and time at 1/60 Hz share one dirty-flag pass;
//   - an app that is not running still has its fields updated;
//   - testing one app means constructing every app's data.
//
// A topic per fact fixes all five: the weather app adds WeatherState and
// nobody else recompiles. See doc-design/app-architecture.md section 4.
//
// STATE, NOT EVENTS -- and this is the line people get wrong
//
//   Topic  = "what is true now"   battery 47%, 3204 steps, 14:32
//            coalescing is CORRECT, missing an update is HARMLESS,
//            order does not matter.
//
//   Queue  = "what just happened" alarm fired, button pressed
//            coalescing LOSES data, order matters, a miss is a BUG.
//
// Putting "alarm fired" in a topic loses the alarm whenever the UI is busy for
// one frame. Putting "battery level" in a queue fills the queue with values
// that were already stale when they were read. Both mechanisms must exist;
// ui.md section 9.1 keeps the queue lanes for the event half.
//
// WHY A GENERATION COUNTER AND NOT A CALLBACK
//
// A callback would run in the PUBLISHER's task, which drags the reader into the
// wrong context -- the exact mistake sample_fanout.hpp warns about. A counter
// lets the UI task ask "anything new?" at its own frame rate, and coalesces for
// free: 200 publishes between two frames cost one read.
#pragma once

#include <cstdint>
#include <type_traits>

namespace core {

// Where a reader keeps its place. Each reader owns one of these PER TOPIC, so
// two apps reading the same topic never affect each other's staleness.
//
// Zero means "has never read". That is what makes an app entering the
// foreground get the current value immediately on its first poll, instead of
// showing a blank until the next publish -- see the firstReadReturnsLatest test.
struct Cursor {
    uint32_t lastSeen{0};
};

namespace detail {

// Advance a generation counter, skipping 0.
//
// Named and pulled out of publish() so the test can exercise the wrap for real
// instead of re-implementing it -- a copy of this rule in the test file would
// keep passing after the original changed, which is worse than no test.
//
// Zero is reserved for "never published". Without the skip, the meaning of the
// counter silently inverts after 2^32 publishes (~497 days at 100 Hz) and every
// reader goes permanently stale.
inline uint32_t nextGeneration(uint32_t g)
{
    ++g;
    if (g == 0) {
        g = 1;
    }
    return g;
}

}  // namespace detail

// Default synchronisation policy: none.
//
// Correct for host tests, and correct on target for a topic that is published
// and read by the same task. Anything crossing a task boundary must use the
// CriticalSection policy from topic_esp.hpp instead -- the ESP32-S3 is
// dual-core, so a publisher and a reader genuinely run at the same instant.
struct NoLock {
    void enter() const {}
    void exit() const {}
};

template <typename T, typename Lock = NoLock>
class Topic {
    static_assert(std::is_trivially_copyable<T>::value,
                  "a topic value is copied inside a critical section: it must "
                  "be a small POD, never something that allocates or owns");

public:
    Topic()                        = default;
    Topic(const Topic&)            = delete;
    Topic& operator=(const Topic&) = delete;

    // Callable from any task, and from an ISR when the policy allows it.
    void publish(const T& v)
    {
        lock_.enter();
        value_      = v;
        generation_ = detail::nextGeneration(generation_);
        lock_.exit();
    }

    // Returns true and fills `out` only when there is something the caller has
    // not seen. Advances the cursor.
    //
    // False means either "never published" or "unchanged since your last read".
    // Those are deliberately not distinguished here: a caller that needs the
    // difference asks hasValue().
    bool read(T& out, Cursor& cursor) const
    {
        lock_.enter();
        const uint32_t g = generation_;
        if (g == 0 || g == cursor.lastSeen) {
            lock_.exit();
            return false;
        }
        out = value_;
        lock_.exit();
        cursor.lastSeen = g;
        return true;
    }

    // Current value regardless of the cursor. For a screen being built that
    // needs to paint something immediately, before its first poll.
    bool peek(T& out) const
    {
        lock_.enter();
        const bool has = (generation_ != 0);
        if (has) {
            out = value_;
        }
        lock_.exit();
        return has;
    }

    bool hasValue() const
    {
        lock_.enter();
        const bool has = (generation_ != 0);
        lock_.exit();
        return has;
    }

    // Publish count, wrapping and skipping 0. Two samples of this taken a
    // second apart give the publish rate; compared against how often a reader
    // actually read, it gives the coalescing ratio -- which is the number that
    // says whether a UI is keeping up.
    uint32_t generation() const
    {
        lock_.enter();
        const uint32_t g = generation_;
        lock_.exit();
        return g;
    }

private:
    mutable Lock lock_{};
    T            value_{};
    uint32_t     generation_{0};
};

}  // namespace core
