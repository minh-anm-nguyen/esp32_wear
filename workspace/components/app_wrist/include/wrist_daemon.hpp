// The background half of the wrist app.
//
// WHY AN APP HAS A BACKGROUND HALF AT ALL
//
// app.hpp rule 4 forbids a background APP from running anything, and that rule
// is what lets a watch with fifteen apps still sleep. But some features
// genuinely have to keep working while their screen is closed -- a statistic
// that only accumulates while you are looking at it is not a statistic.
//
// So the work moves to a half that is a service in every technical sense:
// given a task at boot, event-driven from then on, and structurally unable to
// touch LVGL. What is new is only that it SHIPS WITH its app, in one component,
// instead of being a separate thing somebody has to remember to write and wire
// up somewhere else.
//
// RUNS IN ITS OWN TASK. That is the difference from the first version, which
// ran inside the IMU task at priority 10 -- above the screen, on a driver's
// stack. Now:
//
//   - the stack below is this app's alone; overflowing it cannot corrupt a
//     driver, and the crash report names "wrist.daemon"
//   - the priority is clamped below the UI, so this can never cost a frame
//   - a slow onSample() drops THIS daemon's samples and nobody else's
//
// What has NOT changed: no LVGL, ever. There is no UI task here and no lock
// that would make it safe. See daemon.hpp.
#pragma once

#include "daemon.hpp"
#include "topic_esp.hpp"

#include <cstdint>

namespace apps {

// What the daemon accumulates. Small, POD, copied under a critical section.
struct ActivityState {
    uint32_t activeMs{};      // time spent moving, since boot
    uint32_t sampleCount{};
    float    peakG{};         // largest deviation from rest seen
};

class WristDaemon final : public background::IAppDaemon {
public:
    using Topic = core::Topic<ActivityState, core::CriticalSection>;

    // NO CONSTRUCTOR ARGUMENT any more.
    //
    // It used to take the IMU fan-out, because attaching itself to that stream
    // was the daemon's whole job. The framework owns that wiring now, which
    // removes the one place an app could have got the sensor task's context by
    // accident.
    WristDaemon() = default;

    // Becomes the FreeRTOS task name, so it is what a panic or a watchdog
    // report prints. Under 15 characters on purpose.
    const char* id() const override { return "wrist.daemon"; }

    background::DaemonConfig config() const override
    {
        background::DaemonConfig c{};
        // Measured, not guessed: the log prints this task's high-water mark
        // every ten seconds and warns at 75%. Start generous, then tighten
        // against the number rather than against a feeling.
        c.stackSize = 3072;
        c.priority  = 4;       // below the UI task at 6; the framework enforces it
        // 32, set by measurement rather than by estimate.
        //
        // Eight was the first guess and the board disagreed: 24% of samples
        // dropped, because this task sits below the UI and the UI holds a core
        // for hundreds of milliseconds while it builds a screen or redraws
        // under a moving finger. 32 slots at 62.5 Hz is about half a second of
        // slack, for 1 KB.
        //
        // It does not make the daemon immune, and is not supposed to. What
        // makes THIS daemon correct under loss is below: activeMs accumulates
        // from the gap between sample TIMESTAMPS, so a dropped sample costs a
        // little resolution and no time at all -- which is why its numbers
        // stayed right through a run that lost a quarter of its input.
        c.inboxDepth   = 32;
        c.wantsSamples = true;
        c.tickMs       = 0;    // purely event-driven; the samples are the clock
        return c;
    }

    void onStart() override;
    void onSample(const sensors::Sample& sample, uint32_t nowMs) override;

    const Topic& state() const { return state_; }

private:
    // Movement threshold, in g away from the 1 g the watch feels at rest.
    //
    // A deliberately crude measure, and named so nobody mistakes it for a step
    // count: real pedometry is a product decision that has not been made yet
    // (hardware pedometer in the QMI8658 versus a software algorithm), and
    // inventing one here would be guessing at the answer.
    static constexpr float kMoveThresholdG = 0.15f;

    Topic state_{};

    ActivityState acc_{};
    uint32_t      lastMs_{0};
    uint32_t      published_{0};
};

}  // namespace apps
