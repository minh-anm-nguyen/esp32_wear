// The first inhabitant of the service tier: turns an IMU sample stream into the
// domain fact "the wrist is raised", and publishes it as state.
//
// WHAT A SERVICE IS, and this file is the reference for the ones that follow:
//
//   - owns ONE domain concept, and the policy object that computes it;
//   - subscribes to a driver IN THE DRIVER'S TASK (here: via ISampleSink);
//   - publishes core::Topic<T> that any task may read;
//   - never touches LVGL, never knows an app exists, never owns a driver.
//
// See doc-design/app-architecture.md section 5.
//
// WHY THIS SITS BETWEEN motion AND THE UI
//
// components/motion emits EVENTS: "a raise just happened". A screen being drawn
// does not need the event, it needs the STATE: "is the wrist up right now". An
// app entering the foreground half a second after the raise has missed the
// event entirely and would render a blank -- which is exactly the distinction
// topic.hpp is built around. This class is where one becomes the other.
//
// The event half is not lost: whoever needs "a raise JUST happened" -- waking
// the screen, for instance -- still goes through the callback, because a wake
// that gets coalesced away is a wake that never happened.
//
// PURE ON PURPOSE
//
// Includes nothing but pure headers, so the whole thing runs under g++ in
// host_test/. That only works because the Lock policy is a template parameter:
// main/services.hpp instantiates it with core::CriticalSection for the real
// cross-task build, the tests use the default NoLock.
#pragma once

#include "motion_controller.hpp"
#include "sensor_types.hpp"
#include "topic.hpp"

#include <cstdint>

namespace svc {

// Small, POD, no pointers -- it is copied inside a critical section.
struct WristState {
    bool     raised{false};
    uint32_t sinceMs{0};     // when it last changed
    uint32_t raiseCount{0};  // cheap liveness signal for diagnostics
};

template <typename Lock = core::NoLock>
class WristServiceT final : public sensors::ISampleSink {
public:
    WristServiceT() { attachCallback(); }

    explicit WristServiceT(const motion::MotionConfig& cfg) : motion_(cfg)
    {
        attachCallback();
    }

    // NOT copyable and NOT movable, and this is load-bearing rather than
    // conventional: the constructor hands `this` to MotionController as a
    // callback context. A moved-from copy would leave the controller calling
    // into the old address -- the classic C-callback/C++-lifetime trap, and one
    // that produces no compiler warning whatsoever.
    WristServiceT(const WristServiceT&)            = delete;
    WristServiceT& operator=(const WristServiceT&) = delete;
    WristServiceT(WristServiceT&&)                 = delete;
    WristServiceT& operator=(WristServiceT&&)      = delete;

    // sensors::ISampleSink -- runs in the IMU task, so this must stay short.
    // It does exactly one thing: hand the sample to the policy object and let
    // the callback below do the publishing.
    void onSample(const sensors::Sample& sample, uint32_t nowMs) override
    {
        motion_.onSample(sample, nowMs);
    }

    // What apps read. Const: a reader must not be able to publish.
    const core::Topic<WristState, Lock>& state() const { return state_; }

    // For the composition root to forward raise/lower EVENTS onward (waking the
    // screen). Set before the driver starts; the service keeps publishing its
    // topic either way.
    using EventFn = void (*)(void* ctx, motion::MotionEvent ev, uint32_t nowMs);
    void setEventCallback(EventFn fn, void* ctx)
    {
        userFn_  = fn;
        userCtx_ = ctx;
    }

    const motion::MotionController& motion() const { return motion_; }

private:
    void attachCallback() { motion_.setEventCallback(&WristServiceT::onMotion, this); }

    static void onMotion(void* ctx, motion::MotionEvent ev, uint32_t nowMs)
    {
        auto* self = static_cast<WristServiceT*>(ctx);

        if (ev == motion::MotionEvent::WRIST_RAISE) {
            ++self->raiseCount_;
            self->state_.publish({true, nowMs, self->raiseCount_});
        } else if (ev == motion::MotionEvent::WRIST_LOWER) {
            self->state_.publish({false, nowMs, self->raiseCount_});
        }

        // Forwarded AFTER publishing, so anything the callback triggers already
        // sees the new state. Reversing these two lines makes a screen woken by
        // a raise read the state from before the raise -- and that reads as a
        // random one-frame flicker, which is a horrible thing to debug.
        if (self->userFn_ != nullptr) {
            self->userFn_(self->userCtx_, ev, nowMs);
        }
    }

    motion::MotionController      motion_{};
    core::Topic<WristState, Lock> state_{};
    uint32_t                      raiseCount_{0};

    EventFn userFn_{nullptr};
    void*   userCtx_{nullptr};
};

// Host tests and any same-task use.
using WristService = WristServiceT<core::NoLock>;

}  // namespace svc
