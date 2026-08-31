#include "services.hpp"

#include "esp_log.h"

namespace app {
namespace {
constexpr const char* TAG = "svc";
}

esp_err_t Services::attach(board::Board& b)
{
    // Checked, not assumed. A service that failed to register produces no error
    // at runtime -- it simply never sees data, and the symptom is a screen that
    // is merely always blank.
    if (!b.imuSamples().add(&wrist_)) {
        ESP_LOGE(TAG, "khong dang ky duoc WristService vao luong IMU");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "da gan: WristService");
    return ESP_OK;
}

}  // namespace app
