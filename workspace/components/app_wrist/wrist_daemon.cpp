#include "wrist_daemon.hpp"

#include <cmath>

#include "esp_log.h"

namespace apps {
namespace {
constexpr const char* TAG = "wrist.daemon";

// Longest gap we are willing to count as "still moving". Anything larger means
// the stream stopped -- the IMU went to a low-power mode, or the chip dropped
// out -- and counting that whole gap as activity would invent minutes that
// never happened.
constexpr uint32_t kMaxGapMs = 200;
}  // namespace

void WristDaemon::onStart()
{
    // Runs in this daemon's own task, once, before the first sample arrives.
    //
    // The old version had real work here -- attaching to the IMU fan-out, and
    // an error path for when that failed. The framework does that now, so what
    // is left is a line saying the app's background half is alive, which is
    // worth keeping: a daemon that never starts produces no error at runtime,
    // its numbers simply stay at zero forever.
    ESP_LOGI(TAG, "dang chay trong task rieng");
}

void WristDaemon::onSample(const sensors::Sample& s, uint32_t nowMs)
{
    ++acc_.sampleCount;

    const float mag = std::sqrt(s.accelG[0] * s.accelG[0] + s.accelG[1] * s.accelG[1]
                                + s.accelG[2] * s.accelG[2]);
    const float dev = std::fabs(mag - 1.0f);
    if (dev > acc_.peakG) {
        acc_.peakG = dev;
    }

    const uint32_t gap = (lastMs_ != 0 && nowMs > lastMs_) ? (nowMs - lastMs_) : 0u;
    lastMs_            = nowMs;

    if (dev > kMoveThresholdG && gap > 0 && gap <= kMaxGapMs) {
        acc_.activeMs += gap;
    }

    // Publish once a second, not once a sample.
    //
    // At 62.5 Hz a publish per sample would bump the topic generation sixty
    // times a second so that a reader could be told a millisecond changed. The
    // topic coalesces, so nothing would break -- it would just be work nobody
    // asked for. It no longer costs the SENSOR task anything (this runs in its
    // own task now), but it still takes a critical section every time, and
    // those are shared with everybody.
    if (acc_.activeMs / 1000 != published_ || acc_.sampleCount == 1) {
        published_ = acc_.activeMs / 1000;
        state_.publish(acc_);
    }
}

}  // namespace apps
