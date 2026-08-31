// Shared vocabulary for motion data. Header only, includes nothing but <cstdint>.
//
// WHY THIS COMPONENT EXISTS
//
// `Sample` is spoken by two components that must not depend on each other:
//   - components/imu     produces it from a QMI8658C
//   - components/motion  consumes it to recognise gestures
//
// Putting the type in either one makes the other depend on it, and the whole
// point of splitting them was so that swapping the IMU for a BMI270 does not
// touch the gesture code. A type owned by neither, required by both, is the
// only arrangement in which that claim is actually true.
//
// See doc-design/imu-qmi8658c-design.md section 18.
//
// ONE PRODUCER, MANY CONSUMERS: a driver holds a single ISampleSink*, which is
// all a driver should know about. When several things need the same stream, put
// a sensors::SampleFanout in that one slot -- see sample_fanout.hpp next door,
// and doc-design/app-architecture.md section 3.
#pragma once

#include <cstdint>

namespace sensors {

// One inertial reading, already converted to physical units and already
// rotated into the SCREEN frame -- not the sensor frame. Whoever produces a
// Sample has done the axis remap; whoever consumes one never has to know how
// the chip was glued to the board.
struct Sample {
    float    accelG[3]{};     // g
    float    gyroDps[3]{};    // degrees per second; all zero when the gyro is off
    uint32_t timestampMs{};   // esp_timer converted to ms, wraps every ~49.7 days
};

// The seam between a sensor driver and whatever wants its data stream.
//
// A driver holds a pointer to this and nothing more; it never learns what is on
// the other side. The application is what connects the two -- exactly how
// main.cpp already connects the button to the buzzer without either component
// referencing the other.
class ISampleSink {
public:
    virtual ~ISampleSink() = default;

    // Called from the producer's task, once per sample. Keep it short: it runs
    // in the sensor task's timing budget, not the caller's.
    virtual void onSample(const Sample& sample, uint32_t nowMs) = 0;
};

}  // namespace sensors
