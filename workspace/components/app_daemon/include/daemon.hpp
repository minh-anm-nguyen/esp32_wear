// The background half of an app, with a task of its own.
//
// WHY THIS MOVED OUT OF app_api/app.hpp
//
// The first version of IAppDaemon had one method, onStart(), and its whole job
// was to attach the daemon to something that ALREADY had a task -- normally the
// IMU sample fan-out. That was cheap, and it was wrong for the stated goal:
//
//   - the daemon's code ran in the IMU task, at priority 10, ABOVE the UI.
//     A background half could therefore cost the foreground app its frames.
//   - it ran on the IMU driver's 4096-byte stack. A local buffer in an app's
//     onSample() overflowed a DRIVER's stack, and the crash named the task
//     "imu" -- pointing the finger at the wrong code.
//   - one slow daemon delayed every other daemon and ate into the sensor's
//     sample period, with no counter anywhere that said so.
//
// None of that is compatible with "developers develop freely and one app's
// mistake does not reach another app". So a daemon now gets its OWN task, its
// OWN stack, and a priority the framework clamps BELOW the UI task.
//
// WHAT ISOLATION ACTUALLY BUYS, HONESTLY
//
// There is no MMU on the ESP32-S3 and no memory protection between FreeRTOS
// tasks. So separate tasks contain some faults and not others:
//
//   slow / CPU hog        CONTAINED  -- runs below the UI, which preempts it
//   blocks forever        CONTAINED  -- only that daemon stops; TWDT names it
//   infinite loop         CONTAINED  -- same
//   stack overflow        NOT        -- canary aborts the board, but the task
//                                      name in the panic is now the APP's
//   null / wild pointer   NOT        -- CPU exception, reboot
//
// The second half of that table is not a gap in this design, it is the chip.
// What the design CAN do about it is make the failure name its author, which
// is why every task here is named after its daemon.
#pragma once

#include "sensor_types.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>

namespace background {

// The band a daemon's priority is clamped into.
//
// The ceiling is the point. The ladder is touch 11, IMU 10, button 9, UI 6,
// buzzer 5 (doc-design/i2c-bus-design.md section 11), and a daemon must never
// reach the UI's 6: background work that can cost the foreground a frame is
// exactly the arrangement this component exists to end. An app asking for 12
// is not refused with an error it would have to handle -- it is clamped, and
// told, because a daemon that fails to start is a feature that silently stops
// working.
inline constexpr UBaseType_t kMinPriority = 2;
inline constexpr UBaseType_t kMaxPriority = 5;

// Never sleep longer than this in one go, whatever tickMs says: the task
// watchdog is armed at 5 s (CONFIG_ESP_TASK_WDT_TIMEOUT_S) and a task that
// blocks past it is reported as hung. 2 s leaves room for a late wake-up.
inline constexpr uint32_t kMaxSleepMs = 2000;

struct DaemonConfig {
    // THE APP'S OWN STACK. Nothing else runs on it.
    //
    // Sizing it is the developer's call -- they are the only one who knows what
    // their code puts on it. It is never silent, though: DaemonHost logs the
    // high-water mark of every daemon and warns once when a task has used more
    // than three quarters of what it asked for, which is the point at which
    // "it works on my bench" turns into a field crash.
    uint32_t stackSize{3072};

    // Clamped into [kMinPriority, kMaxPriority]. 4 = below the UI, above the
    // buzzer.
    UBaseType_t priority{4};

    // How many samples may wait between the IMU task and this daemon.
    //
    // Overflow DROPS, and the drop is counted against this daemon. That is the
    // whole trade being bought: a slow daemon now loses its own samples instead
    // of stealing the sensor task's timing budget from everyone else.
    //
    // 16, raised from 8 by measurement. A daemon sits below the UI on purpose,
    // so when the UI is legitimately busy -- building a screen, redrawing under
    // a finger -- this task waits, and the first board run showed it waiting as
    // long as 628 ms. Eight slots at 62.5 Hz is 128 ms of slack, which was not
    // enough: 24% of samples were dropped. Sixteen buys 256 ms for 512 bytes.
    //
    // Buffer alone cannot fix starvation, and is not meant to. A daemon that
    // must not miss anything should be built to survive gaps -- WristDaemon
    // accumulates from sample TIMESTAMPS rather than by counting samples, which
    // is why its numbers stayed right while a quarter of its input vanished.
    uint8_t inboxDepth{16};

    // Periodic wake-up in milliseconds. 0 = purely event-driven.
    uint32_t tickMs{0};

    // Feed onSample()? A daemon that only wants a timer says false and costs no
    // queue at all.
    bool wantsSamples{true};
};

// ---------------------------------------------------------------------------

class IAppDaemon {
public:
    virtual ~IAppDaemon() = default;

    // Short, stable, unique. Becomes the FreeRTOS task name, so it is what a
    // panic or a watchdog report will print. Keep it under 15 characters.
    virtual const char* id() const = 0;

    // Asked once, before the task is created.
    virtual DaemonConfig config() const { return {}; }

    // Runs IN THIS DAEMON'S OWN TASK, once, before the first sample.
    //
    // Unlike the old design this may block, allocate, and talk to a service:
    // it is no longer holding up the boot or sitting in a sensor's timing
    // budget. It must still never touch LVGL -- there is no UI task here and
    // no lock that would make it safe.
    virtual void onStart() {}

    // One sample, in this daemon's own task. Never the IMU's.
    virtual void onSample(const sensors::Sample& sample, uint32_t nowMs) {}

    // Every DaemonConfig::tickMs, if that is non-zero.
    virtual void onTick(uint32_t nowMs) {}
};

}  // namespace background
