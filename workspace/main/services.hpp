// The domain layer: things that turn driver output into facts an app can read.
//
// A service owns ONE concept, subscribes to a driver in the DRIVER's task,
// publishes core::Topic<T> that any task may read, and never touches LVGL or
// knows an app exists. doc-design/app-architecture.md section 5.
//
// WHY SERVICES ARE NOT DRIVERS AND NOT APPS
//
// Drivers run in their own tasks. Apps run in the UI task. Something has to
// cross that boundary exactly once, safely, with coalescing -- and that is a
// service's second job, the one that makes the tier structural rather than
// decorative:
//
//   driver task  --ISampleSink-->  service  --Topic-->  UI task
//
// WHAT IS DELIBERATELY NOT HERE YET
//
// ActivityService (step counting) is blocked on a decision that is a product
// call, not a code call: steps can come from the QMI8658's hardware pedometer
// (ImuEventMsg::Type::STEP, which arrives on a queue only one consumer can
// drain) or from a software algorithm over raw samples. Those have different
// accuracy, different power, and different plumbing. Inventing one here would
// be guessing at the answer.
//
// HapticService (buzz arbitration) has nothing to arbitrate until two apps can
// both ask for a buzz. Building it now would be machinery ahead of need --
// app-architecture.md section 14.
#pragma once

#include "board.hpp"
#include "topic_esp.hpp"
#include "wrist_service.hpp"

#include "esp_err.h"

namespace app {

// The real instantiation: published from the IMU task, read from anywhere, so
// the topic needs a genuine critical section rather than the NoLock the host
// tests use.
using WristService = svc::WristServiceT<core::CriticalSection>;

class Services {
public:
    Services()                           = default;
    Services(const Services&)            = delete;
    Services& operator=(const Services&) = delete;

    // Registers every service with the driver streams it consumes.
    //
    // MUST run between Board::initDevices() and Board::startSensors(): the
    // fan-out is sealed by the second one and add() is unsafe once the IMU task
    // is running. See the two-phase note in board.hpp.
    esp_err_t attach(board::Board& b);

    WristService& wrist() { return wrist_; }

private:
    WristService wrist_{};
};

}  // namespace app
