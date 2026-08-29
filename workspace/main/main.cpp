// Demo for the button + buzzer components.
//
// GPIO0 is the BOOT button on most ESP32-S3 devkits, so this is testable right
// after flashing. GPIO0 is a strapping pin: fine to use, just remember it selects
// the boot mode at reset time (see section 11 of the design document).
//
// The buzzer sits on GPIO42, which is where the V2.1 pinout of this board puts
// it. Boards from the older revision use GPIO33 instead, so check the silkscreen
// before blaming the firmware.
//
// GPIO42 is MTMS, one of the four JTAG pins (39 MTCK, 40 MTDO, 41 MTDI, 42 MTMS).
// That is harmless here because the ESP32-S3 debugs over its built-in USB
// Serial/JTAG on GPIO19/20; it only matters if you wire an external JTAG probe
// to the MTxx pins, which would then fight the buzzer for the pad.
//
// Driver stage: GPIO42 -> base resistor -> SS8050 NPN, buzzer between 3V3 and the
// collector. Active high, so activeLow stays false, and the pad must sit LOW
// whenever no note is playing or the transistor keeps conducting.
#include "button_manager.hpp"
#include "buzzer_manager.hpp"

#include <cinttypes>

#include "driver/ledc.h"   // self-test only: reads the peripheral back
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char* TAG = "app";

constexpr gpio_num_t kButtonPin = GPIO_NUM_0;
constexpr gpio_num_t kBuzzerPin = GPIO_NUM_42;  // GPIO_NUM_33 on the older revision

const char* eventName(button::ButtonEvent e)
{
    switch (e) {
    case button::ButtonEvent::PRESS_DOWN:         return "PRESS_DOWN";
    case button::ButtonEvent::CLICK:              return "CLICK";
    case button::ButtonEvent::DOUBLE_CLICK:       return "DOUBLE_CLICK";
    case button::ButtonEvent::LONG_PRESS:         return "LONG_PRESS";
    case button::ButtonEvent::LONG_PRESS_RELEASE: return "LONG_PRESS_RELEASE";
    case button::ButtonEvent::NONE:               return "NONE";
    }
    return "?";
}

// One gesture, one sound. This table is the entire coupling between the two
// components: neither one references the other, the application joins them.
// Returning a pointer into flash is safe -- the stock patterns are constexpr,
// so they outlive any playback (see the lifetime note on buzzer::Pattern).
const buzzer::Pattern* soundFor(button::ButtonEvent e)
{
    switch (e) {
    // The only sound tied to the pin instead of to a recognised gesture, and so
    // the only one that can be instant: it goes out about 20 ms after the finger
    // lands, which is what makes the button feel responsive.
    case button::ButtonEvent::PRESS_DOWN:   return &buzzer::patterns::kClick;

    // Deliberately silent. PRESS_DOWN already acknowledged this same press some
    // 250 ms earlier; beeping again when the double click window finally expires
    // would just sound like an echo of it.
    case button::ButtonEvent::CLICK:        return nullptr;

    case button::ButtonEvent::DOUBLE_CLICK: return &buzzer::patterns::kOk;

    // kAlarm has repeat = 0 and priority 200. It loops until something stops it,
    // and while it runs a CLICK (priority 10) cannot cut in. Try it: keep
    // clicking during the alarm and nothing happens to the sound.
    case button::ButtonEvent::LONG_PRESS:   return &buzzer::patterns::kAlarm;

    default:                                return nullptr;
    }
}

// One second of continuous tone, played once at boot.
constexpr buzzer::Note kSelfTestNotes[] = {{2000, 1000, 100}};

// Separates the two failure modes that sound identical from the outside:
// firmware not driving the pin at all, versus a pin that carries the right
// square wave into hardware that cannot make a sound out of it.
//
// ledc_get_freq() and ledc_get_duty() read the peripheral registers back, so
// what they print is what the pin is actually doing, not what we asked for.
void buzzerSelfTest(buzzer::BuzzerManager&      buz,
                    const buzzer::BuzzerWiring& wiring,
                    const buzzer::BuzzerSpec&   spec)
{
    ESP_LOGI(TAG, "--- self test: 1 giay am 2000 Hz tren GPIO%d ---",
             static_cast<int>(wiring.pin));

    const esp_err_t err = buz.play(buzzer::makePattern(kSelfTestNotes, 1, 255));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "play() that bai: %s", esp_err_to_name(err));
        return;
    }

    // Let the buzzer task pick the command up and program LEDC before reading.
    vTaskDelay(pdMS_TO_TICKS(200));

    if (spec.type == buzzer::BuzzerType::ACTIVE) {
        ESP_LOGI(TAG, "  loai ACTIVE: muc GPIO = %d (mong doi %d)",
                 gpio_get_level(wiring.pin), wiring.activeLow ? 0 : 1);
    } else {
        const uint32_t freq = ledc_get_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
        const uint32_t duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ESP_LOGI(TAG, "  LEDC doc nguoc: freq=%" PRIu32 " Hz, duty=%" PRIu32 "/1024",
                 freq, duty);

        // volume 100 is clamped to maxVolume, and volume maps onto the lower half
        // of the duty range because a piezo is silent at 100% duty.
        const uint32_t wantDuty = 512u * spec.maxVolume / 100u;
        if (freq == 0 || duty == 0) {
            ESP_LOGE(TAG, "  LEDC KHONG chay -> loi phan mem, khong phai day noi");
        } else if (duty != wantDuty) {
            ESP_LOGW(TAG, "  duty la %" PRIu32 ", mong doi %" PRIu32, duty, wantDuty);
        } else {
            ESP_LOGI(TAG, "  LEDC dung: chan GPIO%d DANG co xung vuong. Neu khong "
                          "nghe thay gi thi van de nam o day noi hoac loa.",
                     static_cast<int>(wiring.pin));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));  // let the tone finish before the demo starts
}

}  // namespace

extern "C" void app_main(void)
{
    // pollIntervalMs derives itself from the tick rate: 10ms at HZ=100, 5ms at HZ=1000.
    static button::ButtonManager btn;
    static buzzer::BuzzerManager buz;

    ESP_LOGI(TAG, "tick rate = %d Hz, 1 tick = %d ms",
             static_cast<int>(configTICK_RATE_HZ),
             static_cast<int>(portTICK_PERIOD_MS));

    // Wiring and behaviour are separate structs on purpose: the first describes
    // this board, the second describes how the button should feel. Only the
    // second one reaches the pure-logic layer.
    button::ButtonWiring bwire{};
    bwire.pin                = kButtonPin;
    bwire.activeLow          = true;   // button wired to GND
    bwire.enableInternalPull = true;
    bwire.enableWakeup       = false;  // no light sleep in this demo yet

    button::ButtonBehavior bhow{};
    bhow.enableDoubleClick = true;   // on, so all the gesture types show up
    bhow.enablePressDown   = true;   // feedback must not wait for doubleClickMs
    bhow.longPressMs       = 800;
    bhow.doubleClickMs     = 250;
    bhow.debounceMs        = 20;     // real time, correct at any tick rate

    buzzer::BuzzerWiring zwire{};
    zwire.pin       = kBuzzerPin;
    zwire.activeLow = false;

    buzzer::BuzzerSpec zspec{};
    zspec.type      = buzzer::BuzzerType::PASSIVE;  // ACTIVE for a self-oscillating
                                                    // buzzer or a vibration motor
    zspec.maxVolume = 80;   // a bare piezo driven flat out at 3.3V is painful up close
    zspec.minFreqHz = 100;
    zspec.maxFreqHz = 10000;

    ESP_ERROR_CHECK(btn.addButton(bwire, bhow));
    ESP_ERROR_CHECK(buz.init(zwire, zspec));

    // Buzzer first: a button already held down at start() is caught by the very
    // first poll, and the sound path has to exist before that event arrives.
    ESP_ERROR_CHECK(buz.start());
    ESP_ERROR_CHECK(btn.start());

    buzzerSelfTest(buz, zwire, zspec);

    ESP_LOGI(TAG, "san sang. Nut GPIO%d, coi GPIO%d",
             static_cast<int>(kButtonPin), static_cast<int>(kBuzzerPin));
    ESP_LOGI(TAG, "  cham xuong   -> bip ngan NGAY (~20ms, chi qua debounce)");
    ESP_LOGI(TAG, "  double click -> hai tieng len giong");
    ESP_LOGI(TAG, "  giu %" PRIu32 "ms   -> bao thuc lap vo han (click luc nay bi bo qua)",
             bhow.longPressMs);
    ESP_LOGI(TAG, "  nha tay      -> tat bao thuc");
    ESP_LOGI(TAG, "luu y: CLICK van bi tre %" PRIu32 " ms vi FSM phai loai tru double "
                  "click; no khong keu, tieng bip da phat luc cham xuong roi",
             bhow.doubleClickMs);

    button::ButtonEventMsg msg;
    while (btn.waitEvent(msg, UINT32_MAX)) {
        // timestampMs      = when the FSM reached its conclusion
        // pressTimestampMs = when the user started pressing
        // The difference between them is the latency of that event type.
        ESP_LOGI(TAG, "GPIO%-2d  %-19s  press=%" PRIu32 " ms  phat=%" PRIu32 " ms  (tre %" PRIu32 " ms)",
                 static_cast<int>(msg.pin), eventName(msg.event),
                 msg.pressTimestampMs, msg.timestampMs,
                 msg.timestampMs - msg.pressTimestampMs);

        // The alarm never ends on its own, so releasing the button is what stops
        // it. silence() outranks nothing -- stop always wins.
        if (msg.event == button::ButtonEvent::LONG_PRESS_RELEASE) {
            buz.silence();
            continue;
        }

        if (const buzzer::Pattern* sound = soundFor(msg.event)) {
            // Returns ESP_ERR_TIMEOUT if the command queue is full, which only
            // happens when someone presses far faster than a pattern can play.
            // Dropping a beep is the right answer there: this loop must not block.
            const esp_err_t err = buz.play(*sound);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "play() bo qua: %s", esp_err_to_name(err));
            }
        }
    }

    ESP_LOGE(TAG, "waitEvent() that bai - queue khong con");
}
