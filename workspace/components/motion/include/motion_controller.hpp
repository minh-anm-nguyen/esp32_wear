// Wrist-raise recognition. Pure logic: it includes <cmath> and the shared
// sensor vocabulary, nothing else. No ESP-IDF, no QMI8658C -- this class does
// not know what kind of chip produced the samples, which is the whole reason
// components/motion and components/imu can be swapped independently.
//
// See doc-design/imu-qmi8658c-design.md section 12.
#pragma once

#include "sensor_types.hpp"

#include <cstdint>

namespace motion {

enum class MotionState : uint8_t {
    ARM_DOWN,  // forearm hanging; nothing pending
    RAISING,   // arm coming up, not yet settled into a viewing pose
    VIEWING,   // looking at the watch; WRIST_RAISE already emitted
};

enum class MotionEvent : uint8_t {
    NONE,
    WRIST_RAISE,  // settled into a viewing pose -- wake the screen
    WRIST_LOWER,  // arm went back down -- the screen may sleep
};

// AXIS CONVENTION, and a promise the AxisRemap must keep:
//
//   +Z out of the screen, towards whoever is looking at it
//   +Y towards 12 o'clock, which on a wrist points along the forearm to the hand
//   +X towards 3 o'clock
//
// An accelerometer at rest measures the REACTION to gravity, so whichever axis
// points at the sky reads +1 g.
//
// WHY TWO ANGLES AND NOT ONE
//
// A first version judged the pose from the screen normal alone. That cannot
// work, because two completely different situations produce the same number:
//
//   arm hanging by the side, glass facing outward .... normal ~horizontal
//   arm held up, glass vertical in front of the eyes . normal ~horizontal
//
// One says "screen off", the other says "screen on". No threshold on that
// single angle can separate them.
//
// What does separate them is the FOREARM. Hanging, it points down; raised to
// read the time, it is horizontal or above. That is the Y axis, so the decision
// needs both:
//
//   armPitch     how far 12 o'clock is above horizontal
//                -90 = hand straight down, 0 = forearm level, +90 = hand up
//   screenPitch  how far the screen normal is above horizontal
//                +90 = glass at the sky, 0 = glass vertical, -90 = glass down
//
// Viewing is "the forearm is not hanging AND the screen is not facing away".
// That one sentence covers every pose a person actually reads a watch in:
// forearm level with the glass tilted up, forearm at 45 degrees, and forearm
// vertical with the glass straight at the eyes.
struct MotionConfig {
    // Two thresholds on the forearm, never one -- a wrist resting exactly on a
    // single boundary would make the screen flicker. Same reason Button's
    // debounce integrator has two edges rather than a midpoint.
    float armPitchViewDeg{-20.0f};  // above this the forearm is up enough to read
    float armPitchDownDeg{-45.0f};  // below this the arm counts as hanging

    // The screen may be tilted well past vertical and still be readable; it
    // only has to not be pointing at the floor. Deliberately generous: this is
    // a veto on "glass facing away", not a second definition of the pose.
    float screenPitchMinDeg{-30.0f};

    // A lift slower than this is not someone checking the time; it is an arm
    // drifting. Without the window, any slow change of posture eventually
    // crosses both thresholds and wakes the screen in a pocket.
    uint32_t raiseWindowMs{1200};

    // Hold the viewing pose this long before believing it. Kills the wake that
    // would otherwise fire as the wrist sweeps THROUGH a viewing angle on its
    // way somewhere else.
    uint32_t settleMs{120};

    // Gravity gate. Outside this band the watch is being accelerated -- walking,
    // a hand swing, a shake -- and the accelerometer is no longer reporting
    // which way is down, so both angles derived from it are meaningless.
    //
    // This is the single most important line in the class. Most home-made
    // wrist-raise detectors are jittery because they trust tilt during motion.
    float minMagnitudeG{0.7f};
    float maxMagnitudeG{1.3f};
};

class MotionController : public sensors::ISampleSink {
public:
    // Plain function pointer plus context rather than another interface: this
    // runs in the sensor task and there is exactly one consumer.
    using EventFn = void (*)(void* ctx, MotionEvent event, uint32_t nowMs);

    MotionController() = default;
    explicit MotionController(const MotionConfig& config) : config_(config) {}

    // THE testable core. Feed it samples in order; it returns at most one event
    // per call. Host tests drive this directly and never touch the callback.
    MotionEvent update(const sensors::Sample& sample, uint32_t nowMs);

    // sensors::ISampleSink. Runs update() and forwards anything it produced.
    // The manager that calls this never learns the class exists.
    void onSample(const sensors::Sample& sample, uint32_t nowMs) override;

    void setEventCallback(EventFn fn, void* ctx)
    {
        eventFn_  = fn;
        eventCtx_ = ctx;
    }

    MotionState state() const { return state_; }

    // From the last ACCEPTED sample. Stale while the gravity gate is rejecting
    // samples -- deliberately, since a rejected sample has no meaningful angle.
    float armPitchDeg() const { return armPitchDeg_; }
    float screenPitchDeg() const { return screenPitchDeg_; }

    // True while the gravity gate is holding samples off.
    bool inMotion() const { return inMotion_; }

    const MotionConfig& config() const { return config_; }

    void reset();

private:
    bool inViewingPose() const;

    MotionConfig config_{};
    EventFn      eventFn_{nullptr};
    void*        eventCtx_{nullptr};

    MotionState state_{MotionState::ARM_DOWN};
    float       armPitchDeg_{-90.0f};
    float       screenPitchDeg_{0.0f};
    bool        inMotion_{false};

    uint32_t raiseStartMs_{0};
    uint32_t inViewSinceMs_{0};
    bool     inViewPose_{false};

    // A lift starts on the EDGE of the forearm rising through armPitchDownDeg,
    // never on the level being above it. Without this a raise that times out
    // drops back to ARM_DOWN while the arm is still up, and the very next
    // sample starts a fresh window; chain enough of those together and an arm
    // drifting up over several seconds eventually fires, which is precisely
    // what raiseWindowMs exists to prevent.
    //
    // Same principle as Button: react to edges, not to levels.
    bool prevArmDown_{true};
};

// Exposed for tests and diagnostics. Both return degrees in [-90, +90].
float armPitchDegFor(const float accelG[3]);
float screenPitchDegFor(const float accelG[3]);
float magnitudeG(const float accelG[3]);

}  // namespace motion
