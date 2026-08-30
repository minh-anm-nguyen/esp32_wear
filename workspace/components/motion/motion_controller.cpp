#include "motion_controller.hpp"

#include <cmath>

namespace motion {

namespace {

constexpr float kRadToDeg = 57.29577951308232f;

// asin() is undefined outside [-1, 1], and rounding can push a normalised
// component a hair past 1.0. An unclamped call returns NaN, and NaN silently
// makes every comparison downstream false -- a failure with no error anywhere.
float pitchFrom(float component, float magnitude)
{
    if (magnitude <= 0.0f) {
        return -90.0f;  // no information; treat as fully lowered
    }
    float r = component / magnitude;
    if (r > 1.0f) r = 1.0f;
    if (r < -1.0f) r = -1.0f;
    return std::asin(r) * kRadToDeg;
}

}  // namespace

float magnitudeG(const float a[3])
{
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

// +Y is 12 o'clock, which on a wrist points along the forearm to the hand.
// Hand straight down gives -1 g on Y, hence -90 degrees.
float armPitchDegFor(const float a[3])
{
    return pitchFrom(a[1], magnitudeG(a));
}

// +Z is out of the screen. Glass at the sky gives +1 g on Z, hence +90.
float screenPitchDegFor(const float a[3])
{
    return pitchFrom(a[2], magnitudeG(a));
}

void MotionController::reset()
{
    state_          = MotionState::ARM_DOWN;
    armPitchDeg_    = -90.0f;
    screenPitchDeg_ = 0.0f;
    inMotion_       = false;
    raiseStartMs_   = 0;
    inViewSinceMs_  = 0;
    inViewPose_     = false;
    // Start armed: an arm hanging by the side is the normal starting position,
    // and the first lift should be recognised.
    prevArmDown_ = true;
}

// "The forearm is not hanging AND the screen is not facing away." Both terms
// are needed: the first alone would wake on an arm raised with the glass turned
// to the floor, the second alone cannot tell a hanging arm from a raised one.
bool MotionController::inViewingPose() const
{
    return armPitchDeg_ > config_.armPitchViewDeg &&
           screenPitchDeg_ > config_.screenPitchMinDeg;
}

MotionEvent MotionController::update(const sensors::Sample& sample, uint32_t nowMs)
{
    const float mag = magnitudeG(sample.accelG);

    // The gravity gate. While the watch is being accelerated the accelerometer
    // is not telling us which way is down, so both angles derived from it are
    // noise. Reject the sample rather than believe it.
    inMotion_ = (mag < config_.minMagnitudeG || mag > config_.maxMagnitudeG);

    if (inMotion_) {
        // Time still passes. A raise that began and then dissolved into a hand
        // swing must still time out, or the FSM would sit in RAISING until the
        // arm happened to settle into a viewing pose much later.
        if (state_ == MotionState::RAISING &&
            (nowMs - raiseStartMs_) > config_.raiseWindowMs) {
            state_      = MotionState::ARM_DOWN;
            inViewPose_ = false;
        }
        return MotionEvent::NONE;
    }

    armPitchDeg_    = armPitchDegFor(sample.accelG);
    screenPitchDeg_ = screenPitchDegFor(sample.accelG);

    const bool armDown        = armPitchDeg_ < config_.armPitchDownDeg;
    const bool risingThrough  = prevArmDown_ && !armDown;
    prevArmDown_              = armDown;

    switch (state_) {
    case MotionState::ARM_DOWN:
        // The EDGE of the forearm coming up past the outer threshold starts a
        // lift, not the level. An arm that is already up stays unarmed until it
        // comes back down. Nothing is emitted here: crossing this angle happens
        // constantly during ordinary movement.
        if (risingThrough) {
            state_        = MotionState::RAISING;
            raiseStartMs_ = nowMs;
            inViewPose_   = false;
        }
        break;

    case MotionState::RAISING: {
        if (armDown) {
            state_      = MotionState::ARM_DOWN;  // put back down mid-lift
            inViewPose_ = false;
            break;
        }

        // Subtraction on uint32_t, never 'nowMs > deadline': the difference
        // stays correct across the ~49.7 day wrap, same rule as Button and
        // Buzzer.
        if ((nowMs - raiseStartMs_) > config_.raiseWindowMs) {
            state_      = MotionState::ARM_DOWN;  // too slow to be a glance
            inViewPose_ = false;
            break;
        }

        if (inViewingPose()) {
            if (!inViewPose_) {
                inViewPose_    = true;
                inViewSinceMs_ = nowMs;  // start the settling clock
            } else if ((nowMs - inViewSinceMs_) >= config_.settleMs) {
                state_ = MotionState::VIEWING;
                return MotionEvent::WRIST_RAISE;
            }
        } else {
            // Left the pose before settling: the wrist was sweeping through,
            // not stopping. Restart the settling clock.
            inViewPose_ = false;
        }
        break;
    }

    case MotionState::VIEWING:
        // Re-arm only once the forearm actually comes back down past the OUTER
        // threshold. Without this the FSM would fire again on every small
        // wobble -- the same job WAIT_RELEASE does in Button after a
        // DOUBLE_CLICK.
        if (armDown) {
            state_      = MotionState::ARM_DOWN;
            inViewPose_ = false;
            return MotionEvent::WRIST_LOWER;
        }
        break;
    }

    return MotionEvent::NONE;
}

void MotionController::onSample(const sensors::Sample& sample, uint32_t nowMs)
{
    const MotionEvent ev = update(sample, nowMs);
    if (ev != MotionEvent::NONE && eventFn_ != nullptr) {
        eventFn_(eventCtx_, ev, nowMs);
    }
}

}  // namespace motion
