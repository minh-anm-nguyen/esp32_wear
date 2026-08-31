// Gives every daemon a task of its own, and keeps app code out of the sensor
// task entirely.
//
// THE SHAPE
//
//   IMU task (prio 10)          DaemonHost::onSample()
//        |                      one memcpy per daemon, never blocks
//        v
//   [ inbox ][ inbox ][ inbox ]  one queue per daemon, overflow drops
//        |        |        |
//        v        v        v
//   task"wrist" task"x"  task"y"  own stack, own priority, clamped below UI
//
// The sensor task's cost is now bounded by the NUMBER of daemons, not by what
// any of them does. Before this, a daemon that spent 3 ms in onSample() spent
// it inside the IMU's 16 ms sample period at priority 10 -- above the screen --
// and no counter anywhere recorded it.
#pragma once

#include "daemon.hpp"
#include "sensor_types.hpp"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <cstdint>

namespace background {

inline constexpr uint8_t kMaxDaemons = 8;

class DaemonHost final : public sensors::ISampleSink {
public:
    DaemonHost()                             = default;
    DaemonHost(const DaemonHost&)            = delete;
    DaemonHost& operator=(const DaemonHost&) = delete;

    // Register a daemon. Before start(), never after.
    //
    // [[nodiscard]] for the reason AppRegistry::add() is: a daemon that fails
    // to register produces no error at runtime, its numbers simply stay at zero
    // forever, and "the feature quietly does nothing" does not read as a bug.
    [[nodiscard]] bool add(IAppDaemon* d);

    // Creates one task and one inbox per daemon.
    //
    // Partial failure is normal here, exactly as it is in Board: a daemon whose
    // task will not start is logged and skipped, and every other daemon still
    // runs. Returns ESP_OK when at least the host itself is usable.
    esp_err_t start();

    // Runs in the PRODUCER's task. Copies into each inbox and returns; it never
    // blocks, never allocates, and never calls app code.
    void onSample(const sensors::Sample& sample, uint32_t nowMs) override;

    void logDiagnostics(const char* why);

    uint8_t count() const { return count_; }
    uint8_t running() const;

private:
    struct Envelope {
        sensors::Sample sample;
        uint32_t        nowMs;
    };

    struct Slot {
        IAppDaemon*   daemon{nullptr};
        DaemonConfig  cfg{};
        TaskHandle_t  task{nullptr};
        QueueHandle_t inbox{nullptr};
        DaemonHost*   host{nullptr};

        uint32_t received{0};
        uint32_t dropped{0};
        uint32_t ticks{0};

        // WALL CLOCK across the callback, NOT the callback's own cost.
        //
        // This task runs below the UI, so anything that preempts it lands in
        // this number. The first board run reported 628318 us for an onSample()
        // that does one sqrt -- which was not a slow callback, it was 628 ms of
        // being preempted while inside one.
        //
        // Kept, because "how long can this daemon be held up" is worth knowing.
        // Renamed in the log so it stops being read as the app's cost.
        uint32_t maxWallUs{0};

        // THE METRIC THAT SEPARATES THE TWO CASES: how old a sample is by the
        // time the daemon gets to it. Slow callback and starved task both grow
        // the queue, but only starvation grows the age while the callback stays
        // cheap. Same idea as the touch layer's "tuoi mau".
        uint32_t maxAgeMs{0};
        uint32_t ageSumMs{0};
        uint32_t ageCount{0};

        bool     stackWarned{false};
    };

    static void trampoline(void* arg);
    void        taskBody(Slot& slot);

    Slot    slots_[kMaxDaemons]{};
    uint8_t count_{0};
    bool    started_{false};
};

}  // namespace background
