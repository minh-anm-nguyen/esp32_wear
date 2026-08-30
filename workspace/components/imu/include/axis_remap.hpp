// Sensor frame -> screen frame, and the calibration that derives it.
//
// Pure: <cstdint> and nothing else. The derivation is the part worth testing,
// so it lives here rather than inside the interactive procedure that collects
// the measurements.
#pragma once

#include <cstdint>

namespace imu {

// SCREEN FRAME, the convention everything downstream is written against:
//
//   +X  towards 3 o'clock on the watch face
//   +Y  towards 12 o'clock
//   +Z  OUT of the screen, towards whoever is looking at it
//
// A watch lying face up therefore reads accelG = (0, 0, +1): an accelerometer
// at rest measures the REACTION to gravity, so the axis pointing at the sky
// reads +1 g. motion::MotionController's thresholds assume exactly this.
struct AxisRemap {
    int8_t map[3]{0, 1, 2};   // map[i] = which SENSOR axis feeds screen axis i
    int8_t sign[3]{1, 1, 1};

    void apply(const float in[3], float out[3]) const
    {
        for (int i = 0; i < 3; ++i) {
            out[i] = static_cast<float>(sign[i]) * in[map[i]];
        }
    }

    // A valid remap is a signed permutation: each sensor axis used exactly once.
    bool isValid() const
    {
        bool seen[3] = {false, false, false};
        for (int i = 0; i < 3; ++i) {
            if (map[i] < 0 || map[i] > 2 || seen[map[i]]) {
                return false;
            }
            seen[map[i]] = true;
            if (sign[i] != 1 && sign[i] != -1) {
                return false;
            }
        }
        return true;
    }
};

// ------------------------------------------------------------- AxisCalibrator

// Derives an AxisRemap from three measurements.
//
// The idea is one line: hold the watch so a chosen SCREEN axis points at the
// sky, and the accelerometer reads +1 g on whichever SENSOR axis that turned
// out to be. Do it for X, Y and Z and the whole signed permutation falls out.
//
// Three poses, not six. The opposite three would only repeat the same
// information; the checks in derive() catch a sloppily held watch far more
// cheaply than three more prompts do.
class AxisCalibrator {
public:
    enum class Pose : uint8_t {
        ScreenUp = 0,  // face up on a table       -> screen +Z at the sky
        TopUp    = 1,  // on its edge, 12 o'clock up -> screen +Y at the sky
        RightUp  = 2,  // on its edge, 3 o'clock up  -> screen +X at the sky
        Count    = 3,
    };

    // A held pose must look like gravity and nothing else: one component
    // dominant, the other two small. Loose enough for a hand on a table,
    // tight enough to reject a watch held at an angle.
    static constexpr float kMinDominant = 0.80f;
    static constexpr float kMaxOther    = 0.40f;
    static constexpr float kMinMagnitude = 0.85f;
    static constexpr float kMaxMagnitude = 1.15f;

    void reset();

    // Averaged reading for one pose, in the SENSOR frame. Returns false if the
    // vector does not look like a clean 1 g along one axis -- the caller should
    // ask for that pose again rather than derive nonsense from it.
    bool setPose(Pose pose, const float sensorAccel[3]);

    bool hasPose(Pose pose) const { return have_[static_cast<uint8_t>(pose)]; }
    bool complete() const;

    // Solves for the remap. Fails if a pose is missing, or if two poses picked
    // the same sensor axis -- which means the watch was not actually turned
    // between them.
    bool derive(AxisRemap& out) const;

private:
    // Index of the largest |component|, or -1 if the vector is not a clean 1 g
    // along a single axis.
    static int dominantAxis(const float v[3]);

    int8_t axis_[3]{-1, -1, -1};  // sensor axis found for each screen axis
    int8_t sign_[3]{0, 0, 0};
    bool   have_[3]{false, false, false};
};

}  // namespace imu
