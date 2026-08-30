// Self-proving bring-up for the panel, in the same spirit as buzzerSelfTest()
// and ImuManager::runSelfTest(): the firmware demonstrates what the hardware is
// actually doing instead of leaving it to a meter.
//
// This board does NOT wire MISO, so the firmware cannot read GRAM back. Every
// colour and orientation check here is therefore a human one -- the code says
// what should appear, and you confirm it. What the code CAN prove on its own is
// the DMA completion path, so that part is checked automatically.
//
// See doc-design/display.md section 14.
#pragma once

#include "display_manager.hpp"

namespace display {

struct SelfTestResult {
    bool     bufferAllocated{false};
    bool     callbackFired{false};   // proved by firmware, not by eye
    uint32_t transfersCompleted{0};
    bool     brightnessSweepOk{false};
    bool     sleepWakeOk{false};
    uint32_t sleepWakeCycles{0};
};

// holdMs is how long each visual pattern stays on screen. Blocks for roughly
// 10 * holdMs plus the sleep/wake cycles, and must be called from the task that
// owns the DisplayManager.
esp_err_t runSelfTest(DisplayManager& dm, SelfTestResult& out, uint32_t holdMs = 1500);

}  // namespace display
