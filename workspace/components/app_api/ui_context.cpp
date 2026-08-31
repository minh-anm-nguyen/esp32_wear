#include "ui_context.hpp"

#include "esp_log.h"

namespace ui {
namespace {

constexpr const char* TAG = "ui";

TaskHandle_t g_uiTask{nullptr};
bool         g_warned{false};

}  // namespace

void setUiTask(TaskHandle_t task)
{
    g_uiTask = task;
    g_warned = false;
}

bool inUiContext()
{
    return g_uiTask != nullptr && xTaskGetCurrentTaskHandle() == g_uiTask;
}

void assertUiContext(const char* where)
{
    // Before start() there is no UI task and app_main legitimately builds
    // things. Silence then is correct, not a missed violation.
    if (g_uiTask == nullptr || g_warned) {
        return;
    }
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    if (self == g_uiTask) {
        return;
    }

    g_warned = true;
    ESP_LOGE(TAG,
             "%s goi tu task '%s', khong phai UI task '%s'. LV_OS_NONE nen "
             "lv_lock() la no-op: khong deadlock, khong assert, chi hong cay "
             "object roi reboot ngau nhien rat lau sau. Xem "
             "doc-design/app-architecture.md muc 9.",
             where, pcTaskGetName(self), pcTaskGetName(g_uiTask));
}

}  // namespace ui
