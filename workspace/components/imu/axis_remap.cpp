#include "axis_remap.hpp"

#include <cmath>

namespace imu {

void AxisCalibrator::reset()
{
    for (int i = 0; i < 3; ++i) {
        axis_[i] = -1;
        sign_[i] = 0;
        have_[i] = false;
    }
}

int AxisCalibrator::dominantAxis(const float v[3])
{
    const float mag = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (mag < kMinMagnitude || mag > kMaxMagnitude) {
        return -1;  // not gravity alone: the watch was moving, or in free fall
    }

    int   best     = 0;
    float bestAbs  = std::fabs(v[0]);
    for (int i = 1; i < 3; ++i) {
        const float a = std::fabs(v[i]);
        if (a > bestAbs) {
            bestAbs = a;
            best    = i;
        }
    }

    if (bestAbs < kMinDominant) {
        return -1;  // held at an angle: no axis is really pointing up
    }
    for (int i = 0; i < 3; ++i) {
        if (i != best && std::fabs(v[i]) > kMaxOther) {
            return -1;  // a second axis carries too much: tilted about two axes
        }
    }
    return best;
}

bool AxisCalibrator::setPose(Pose pose, const float sensorAccel[3])
{
    const int idx = dominantAxis(sensorAccel);
    if (idx < 0) {
        return false;
    }

    const uint8_t slot = static_cast<uint8_t>(pose);
    axis_[slot] = static_cast<int8_t>(idx);
    // The screen axis was pointing at the SKY, so the sensor axis it maps to
    // reads +1 g. A negative reading means the mapping carries a minus sign.
    sign_[slot] = (sensorAccel[idx] >= 0.0f) ? 1 : -1;
    have_[slot] = true;
    return true;
}

bool AxisCalibrator::complete() const
{
    return have_[0] && have_[1] && have_[2];
}

bool AxisCalibrator::derive(AxisRemap& out) const
{
    if (!complete()) {
        return false;
    }

    AxisRemap r{};
    // Pose ScreenUp resolves screen +Z, TopUp resolves +Y, RightUp resolves +X.
    r.map[2]  = axis_[static_cast<uint8_t>(Pose::ScreenUp)];
    r.sign[2] = sign_[static_cast<uint8_t>(Pose::ScreenUp)];
    r.map[1]  = axis_[static_cast<uint8_t>(Pose::TopUp)];
    r.sign[1] = sign_[static_cast<uint8_t>(Pose::TopUp)];
    r.map[0]  = axis_[static_cast<uint8_t>(Pose::RightUp)];
    r.sign[0] = sign_[static_cast<uint8_t>(Pose::RightUp)];

    // Two screen axes claiming the same sensor axis means the watch was not
    // actually re-oriented between those poses. Silently accepting that would
    // produce a remap that drops one axis entirely.
    if (!r.isValid()) {
        return false;
    }

    out = r;
    return true;
}

}  // namespace imu
