// One producer, many consumers. Header only, includes nothing but <cstdint>
// and its own sibling -- same discipline as sensor_types.hpp, and for the same
// reason: this must stay compilable by a bare g++.
//
// WHY THIS EXISTS
//
// imu::ImuManager::Config holds ONE sensors::ISampleSink*. That was right while
// exactly one thing consumed the stream (MotionController, recognising wrist
// raises). It stops being right at the second consumer, and a watch grows
// several: step counting, sleep tracking, a compass, gesture recognition, a
// raw-data diagnostics screen -- all wanting the same samples.
//
// This class IS an ISampleSink and forwards to N of them. So the driver still
// sees exactly one sink and components/imu does not change by a single line.
// That is the mark of the seam being in the right place: adding consumers is
// not the driver's problem, and it never becomes the driver's problem.
//
// See doc-design/app-architecture.md section 3.
//
// WHAT THIS IS NOT
//
// Not thread-safe, and deliberately not. add() mutates the array that
// onSample() walks, so registering a sink while the producer task is running is
// a data race. The intended shape is: wire everything up in the composition
// root, seal(), then start the driver. seal() is a guard rail that turns a
// misuse into a returned false -- it is NOT a synchronisation primitive and
// does not make concurrent add() safe.
#pragma once

#include "sensor_types.hpp"

#include <cstdint>

namespace sensors {

// Non-template face of the fan-out.
//
// Exists so a subscriber can be handed "somewhere to register" without also
// being told how many slots it has. An app daemon in its own component should
// not have to name SampleFanout<4> -- that number belongs to the composition
// root, and changing it should not touch a single app.
class ISampleFanout : public ISampleSink {
public:
    virtual bool add(ISampleSink* sink) = 0;
    virtual uint8_t count() const       = 0;
    virtual uint8_t capacity() const    = 0;
};

template <uint8_t N>
class SampleFanout final : public ISampleFanout {
    static_assert(N > 0, "a fan-out with no room for a sink is a silent hole");

public:
    // Returns false -- never asserts, never silently succeeds -- when the sink
    // is null, the array is full, this is a duplicate, or seal() has been
    // called. The caller is expected to check: a consumer that failed to
    // register produces no error at runtime, it just never sees data, and that
    // is a very quiet way to lose a step counter.
    bool add(ISampleSink* sink) override
    {
        if (sink == nullptr || sealed_ || count_ >= N) {
            return false;
        }
        // Rejecting duplicates is not tidiness. Adding the same sink twice
        // delivers every sample to it twice, which for anything that
        // ACCUMULATES -- a step counter above all -- silently doubles the
        // result. A wrong step count is much harder to notice than a missing
        // one.
        for (uint8_t i = 0; i < count_; ++i) {
            if (sinks_[i] == sink) {
                return false;
            }
        }
        sinks_[count_++] = sink;
        return true;
    }

    // Freeze the subscriber list. Call once, from the composition root, right
    // before starting the producer. After this add() returns false.
    void seal() { sealed_ = true; }

    bool     isSealed() const { return sealed_; }
    uint8_t  count() const override { return count_; }
    uint8_t  capacity() const override { return N; }
    uint32_t forwarded() const { return forwarded_; }

    // Runs in the PRODUCER's task, and every sink inherits that context: the
    // IMU task at priority 10, not the caller's. The budget is shared, so each
    // onSample() must be short -- no I2C, no filesystem, no LVGL, no blocking,
    // no allocation. A single slow consumer delays every other consumer and
    // eats into the sensor task's sample period.
    //
    // This is exactly why neither half of an app subscribes here directly. The
    // UI half runs in the UI task; the background half runs in a task of its
    // own. What subscribes is a FRAMEWORK sink -- a service, or the single
    // background::DaemonHost that stands in front of every daemon and does
    // nothing here but copy the sample into one queue per daemon.
    //
    // So the cost this loop carries is bounded by the number of subscribers,
    // never by what any app chose to compute. app-architecture.md section 5.
    void onSample(const Sample& sample, uint32_t nowMs) override
    {
        ++forwarded_;
        for (uint8_t i = 0; i < count_; ++i) {
            sinks_[i]->onSample(sample, nowMs);
        }
    }

private:
    ISampleSink* sinks_[N]{};
    uint8_t      count_{0};
    bool         sealed_{false};

    // Wraps at ~4 billion; only ever read as "is anything flowing" and as a
    // delta between two diagnostics dumps, so the wrap is harmless.
    uint32_t forwarded_{0};
};

}  // namespace sensors
