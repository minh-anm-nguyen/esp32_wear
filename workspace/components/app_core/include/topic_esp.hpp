// The ESP-IDF half of core::Topic: the one policy that makes a topic safe to
// publish from one task and read from another.
//
// Split out of topic.hpp on purpose, and for the same reason every other
// component in this repo is split: topic.hpp must stay compilable by a bare
// g++ so the generation protocol -- which is where the bugs are -- can be
// tested on a PC. This file is the three lines that cannot.
//
// See doc-design/app-architecture.md section 4.
#pragma once

#include "topic.hpp"

#include "freertos/FreeRTOS.h"

namespace core {

// A FreeRTOS spinlock, held for exactly as long as it takes to copy a small
// POD -- tens of nanoseconds.
//
// WHY A SPINLOCK AND NOT A MUTEX
//
// The critical section is shorter than the cost of taking a mutex, a mutex
// cannot be taken from an ISR, and a mutex introduces priority inheritance into
// a path that has no business blocking anyone. portMUX also does the part a
// plain interrupt disable would miss: it excludes the OTHER core, and on a
// dual-core ESP32-S3 a publisher on core 1 really can run while the UI task
// reads on core 0.
//
// _SAFE is the variant that works from both task and ISR context. A driver ISR
// publishing straight into a topic is unusual but legal, and picking the
// non-SAFE macro would turn that into a crash a long way from its cause.
class CriticalSection {
public:
    CriticalSection()                                  = default;
    CriticalSection(const CriticalSection&)            = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;

    void enter() const { portENTER_CRITICAL_SAFE(&mux_); }
    void exit() const { portEXIT_CRITICAL_SAFE(&mux_); }

private:
    mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};

// The type to use for anything crossing a task boundary, which in practice is
// every topic a service publishes and an app reads.
//
//   static core::SharedTopic<WristState> wristTopic;   // service publishes
//   core::Cursor cursor;                               // each reader owns one
//   if (wristTopic.read(state, cursor)) { ... }        // UI task
template <typename T>
using SharedTopic = Topic<T, CriticalSection>;

}  // namespace core
