// Demo for the button component.
// GPIO0 is the BOOT button on most ESP32-S3 devkits, so this is testable right
// after flashing. GPIO0 is a strapping pin: fine to use, just remember it selects
// the boot mode at reset time (see section 11 of the design document).
#include "button_manager.hpp"

#include <cinttypes>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "app";

const char* eventName(button::ButtonEvent e)
{
    switch (e) {
    case button::ButtonEvent::CLICK:              return "CLICK";
    case button::ButtonEvent::DOUBLE_CLICK:       return "DOUBLE_CLICK";
    case button::ButtonEvent::LONG_PRESS:         return "LONG_PRESS";
    case button::ButtonEvent::LONG_PRESS_RELEASE: return "LONG_PRESS_RELEASE";
    case button::ButtonEvent::NONE:               return "NONE";
    }
    return "?";
}

}  // namespace

extern "C" void app_main(void)
{
    // pollIntervalMs derives itself from the tick rate: 10ms at HZ=100, 5ms at HZ=1000.
    static button::ButtonManager mgr;

    ESP_LOGI(TAG, "tick rate = %d Hz, 1 tick = %d ms",
             static_cast<int>(configTICK_RATE_HZ),
             static_cast<int>(portTICK_PERIOD_MS));

    button::ButtonConfig cfg{};
    cfg.pin               = GPIO_NUM_0;
    cfg.activeLow         = true;   // button wired to GND
    cfg.enableInternalPull = true;
    cfg.enableDoubleClick = true;   // on, so all four event types show up
    cfg.longPressMs       = 800;
    cfg.doubleClickMs     = 250;
    cfg.debounceMs        = 20;     // real time, correct at any tick rate

    ESP_ERROR_CHECK(mgr.addButton(cfg));
    ESP_ERROR_CHECK(mgr.start());

    ESP_LOGI(TAG, "san sang. Bam GPIO%d: click / double click / giu 800ms",
             static_cast<int>(cfg.pin));
    ESP_LOGI(TAG, "luu y: bat double click nen CLICK bi tre dung %" PRIu32 " ms",
             cfg.doubleClickMs);

    button::ButtonEventMsg msg;
    while (mgr.waitEvent(msg, UINT32_MAX)) {
        // timestampMs      = when the FSM reached its conclusion
        // pressTimestampMs = when the user started pressing
        // The difference between them is the latency of that event type.
        ESP_LOGI(TAG, "GPIO%-2d  %-19s  press=%" PRIu32 " ms  phat=%" PRIu32 " ms  (tre %" PRIu32 " ms)",
                 static_cast<int>(msg.pin), eventName(msg.event),
                 msg.pressTimestampMs, msg.timestampMs,
                 msg.timestampMs - msg.pressTimestampMs);
    }

    ESP_LOGE(TAG, "waitEvent() that bai - queue khong con");
}
