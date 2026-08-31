// Which task is allowed to touch LVGL, and a way to catch the moment something
// else does.
//
// WHY THIS IS NEEDED AT ALL
//
// CONFIG_LV_OS_NONE=y makes lv_lock()/lv_unlock() no-ops, which is correct --
// exactly one task owns the object tree, so a lock would guard nothing. The
// price is that there is NO safety net: an LVGL call from the IMU task, a
// service callback, or the diagnostics loop does not deadlock and does not
// assert. It corrupts the tree and reboots at random, much later, somewhere
// else entirely.
//
// So the rule gets a runtime check, the same way i2c::Device checks its owning
// task and TouchManager checks its single consumer. Those two precedents are
// why this shape: warn LOUDLY, warn ONCE, never abort. A violated invariant
// repeated at 60 Hz buries the message it is trying to draw attention to, and
// aborting a watch in a user's hands is worse than a wrong pixel.
//
// doc-design/app-architecture.md section 9.
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace ui {

// Called by UiManager::start() once the UI task exists.
void setUiTask(TaskHandle_t task);

// False before the UI task has started.
bool inUiContext();

// Logs one error naming both tasks the first time it is violated. `where`
// should name the function, so the log points at the caller and not at this
// file.
void assertUiContext(const char* where);

}  // namespace ui
