// The interactive half of axis calibration: prompt, wait for stillness,
// average, and print a literal to paste. The arithmetic it feeds lives in
// axis_remap.cpp, where it can be tested on a PC.
#include "imu_manager.hpp"

#include <cmath>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace imu {

namespace {

constexpr const char* TAG = "imu-cal";

struct PoseSpec {
    AxisCalibrator::Pose pose;
    const char*          prompt;
    const char*          meaning;
};

// Three poses, each pointing one SCREEN axis at the sky. Which sensor axis then
// reads +1 g is the answer for that row of the remap.
constexpr PoseSpec kPoses[] = {
    {AxisCalibrator::Pose::ScreenUp,
     "Dat dong ho NAM NGUA tren ban, man hinh huong len troi",
     "truc man hinh +Z"},
    {AxisCalibrator::Pose::TopUp,
     "Dung dong ho tren canh, huong 12 gio LEN TREN",
     "truc man hinh +Y"},
    {AxisCalibrator::Pose::RightUp,
     "Dung dong ho tren canh, huong 3 gio LEN TREN",
     "truc man hinh +X"},
};

constexpr uint32_t kPollMs        = 50;

// 0.6 s of stillness, 0.12 g of drift. Two of the three poses need the board
// stood on its edge, which on a module this small means holding it -- and a
// hand cannot hold 0.08 g over a full second. The strict test is not here
// anyway: AxisCalibrator::setPose() still demands one axis above 0.80 g with
// the others under 0.40, and that is what actually protects the result.
constexpr uint32_t kStableSamples = 12;
constexpr float    kStableDeltaG  = 0.12f;

constexpr uint32_t kPoseTimeoutMs = 30000;
constexpr uint32_t kSettleMs      = 1500;  // let the hand settle before sampling

// Say something every second while waiting. Thirty seconds of silence gives
// the operator no way to tell "still waiting for you" from "hung".
constexpr uint32_t kProgressEveryMs = 1000;

// Waits until the watch has been still for kStableSamples in a row, then
// returns the average of that window.
bool captureStillAverage(ImuManager& mgr, float out[3])
{
    const int64_t deadline = esp_timer_get_time() +
                             static_cast<int64_t>(kPoseTimeoutMs) * 1000;

    float    window[3] = {0.0f, 0.0f, 0.0f};
    float    first[3]  = {0.0f, 0.0f, 0.0f};
    uint32_t stable    = 0;

    int64_t nextReport = esp_timer_get_time();

    while (esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(kPollMs));

        sensors::Sample s{};
        if (!mgr.latest(s)) {
            continue;  // nothing published yet
        }

        const float mag = std::sqrt(s.accelG[0] * s.accelG[0] +
                                    s.accelG[1] * s.accelG[1] +
                                    s.accelG[2] * s.accelG[2]);

        // Tell the operator what is being seen and why it is not accepted.
        if (esp_timer_get_time() >= nextReport) {
            nextReport = esp_timer_get_time() +
                         static_cast<int64_t>(kProgressEveryMs) * 1000;
            ESP_LOGI(TAG, "    dang doc (%.2f, %.2f, %.2f) g  |a|=%.2f  -- %s",
                     s.accelG[0], s.accelG[1], s.accelG[2], mag,
                     (mag < 0.85f || mag > 1.15f) ? "con dang chuyen dong"
                                                  : "giu yen them chut nua");
        }

        if (mag < 0.85f || mag > 1.15f) {
            stable = 0;  // still being moved: the reading is not gravity alone
            continue;
        }

        if (stable == 0) {
            for (int i = 0; i < 3; ++i) {
                first[i]  = s.accelG[i];
                window[i] = s.accelG[i];
            }
            stable = 1;
            continue;
        }

        // Every sample in the window must stay close to the first one. A hand
        // slowly rotating the watch would otherwise average into a vector that
        // points nowhere in particular.
        bool drifted = false;
        for (int i = 0; i < 3; ++i) {
            if (std::fabs(s.accelG[i] - first[i]) > kStableDeltaG) {
                drifted = true;
                break;
            }
        }
        if (drifted) {
            stable = 0;
            continue;
        }

        for (int i = 0; i < 3; ++i) {
            window[i] += s.accelG[i];
        }
        if (++stable >= kStableSamples) {
            for (int i = 0; i < 3; ++i) {
                out[i] = window[i] / static_cast<float>(stable);
            }
            return true;
        }
    }
    return false;
}

const char* axisName(int8_t idx)
{
    switch (idx) {
    case 0:  return "X";
    case 1:  return "Y";
    case 2:  return "Z";
    default: return "?";
    }
}

}  // namespace

esp_err_t runAxisCalibration(ImuManager& manager, AxisRemap& out)
{
    if (!manager.isRunning()) {
        ESP_LOGE(TAG, "IMU chua chay -- goi start() truoc");
        return ESP_ERR_INVALID_STATE;
    }

    // latest() hands back samples that have ALREADY been through the remap, so
    // a non-identity remap would make this derive the identity and look like a
    // success. Refuse rather than produce a confidently wrong answer.
    const AxisRemap current = manager.remap();
    const AxisRemap identity{};
    for (int i = 0; i < 3; ++i) {
        if (current.map[i] != identity.map[i] || current.sign[i] != identity.sign[i]) {
            ESP_LOGE(TAG, "chi hieu chuan duoc khi remap con la identity "
                          "(hien tai da co remap -> ket qua se vo nghia)");
            return ESP_ERR_INVALID_STATE;
        }
    }

    ESP_LOGI(TAG, "=== HIEU CHUAN TRUC (3 tu the) ===");
    ESP_LOGI(TAG, "Khong can do gi. Chi xoay dong ho theo huong dan roi GIU YEN.");

    AxisCalibrator cal;

    for (const auto& spec : kPoses) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, ">>> %s", spec.prompt);
        ESP_LOGI(TAG, "    (dang do %s) -- dat xuong roi GIU YEN", spec.meaning);

        // Give the operator time to read the prompt, move the watch and let go.
        vTaskDelay(pdMS_TO_TICKS(kSettleMs));

        float avg[3] = {0.0f, 0.0f, 0.0f};
        if (!captureStillAverage(manager, avg)) {
            ESP_LOGE(TAG, "    het gio: khong giu yen du lau");
            return ESP_ERR_TIMEOUT;
        }

        if (!cal.setPose(spec.pose, avg)) {
            // dominantAxis() refused it: no single axis was really pointing up.
            ESP_LOGE(TAG, "    tu the khong hop le: doc duoc (%.2f, %.2f, %.2f) g. "
                          "Can dung MOT truc huong thang len -- dat lai cho phang.",
                     avg[0], avg[1], avg[2]);
            return ESP_ERR_INVALID_RESPONSE;
        }

        ESP_LOGI(TAG, "    OK: (%.2f, %.2f, %.2f) g", avg[0], avg[1], avg[2]);
    }

    if (!cal.derive(out)) {
        // Two poses landed on the same sensor axis, which means the watch was
        // not actually turned between them.
        ESP_LOGE(TAG, "khong suy ra duoc phep hoan vi hop le -- co ve hai tu the "
                      "bi lam giong nhau. Chay lai.");
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== KET QUA -- dan doan nay vao main.cpp ===");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "    imu::AxisRemap remap{};");
    ESP_LOGI(TAG, "    remap.map[0]  = %d;  remap.sign[0]  = %d;   // man hinh +X <- cam bien %s%s",
             out.map[0], out.sign[0], out.sign[0] < 0 ? "-" : "+", axisName(out.map[0]));
    ESP_LOGI(TAG, "    remap.map[1]  = %d;  remap.sign[1]  = %d;   // man hinh +Y <- cam bien %s%s",
             out.map[1], out.sign[1], out.sign[1] < 0 ? "-" : "+", axisName(out.map[1]));
    ESP_LOGI(TAG, "    remap.map[2]  = %d;  remap.sign[2]  = %d;   // man hinh +Z <- cam bien %s%s",
             out.map[2], out.sign[2], out.sign[2] < 0 ? "-" : "+", axisName(out.map[2]));
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Gan vao icfg.remap roi nap lai. Sau do nang co tay se nhan dung.");

    return ESP_OK;
}

}  // namespace imu
