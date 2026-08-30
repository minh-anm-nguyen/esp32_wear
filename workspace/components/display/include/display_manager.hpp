// ESP-IDF integration layer: SPI2 + esp_lcd ST7789 panel + one LEDC channel for
// the backlight. No task and no queue, unlike ButtonManager and BuzzerManager.
//
// WHY NO TASK
//
// SPI DMA is already asynchronous, and LVGL requires exactly one task to own the
// UI. A display task would add a copy, a context switch and a deadlock
// opportunity while owning nothing the UI task does not already own.
//
// The consequence is that this class is DELIBERATELY NOT THREAD-SAFE. There is
// no mutex anywhere in it. Every method must be called from the UI task and
// only from the UI task; other tasks post a ui::UiCommand instead. That is the
// same rule as section 7.1 of doc-design/i2c-bus-design.md -- one task owns one
// device -- applied to SPI rather than to I2C.
//
// See doc-design/display.md sections 2, 7.2 and 8.
#pragma once

#include "display.hpp"

#include <atomic>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace display {

struct Wiring {
    spi_host_device_t spiHost{SPI2_HOST};
    gpio_num_t sclk{GPIO_NUM_6};
    gpio_num_t mosi{GPIO_NUM_7};
    gpio_num_t dc{GPIO_NUM_4};
    gpio_num_t cs{GPIO_NUM_5};
    gpio_num_t reset{GPIO_NUM_8};
    gpio_num_t backlight{GPIO_NUM_15};
};

struct Config {
    Geometry geometry{};

    // 40 MHz is past the ST7789 datasheet write cycle (~15 MHz) but is what the
    // official Waveshare example for this board uses. First suspect if stray
    // pixels appear: drop to 20 MHz to rule it out.
    uint32_t spiClockHz{40'000'000};

    // MUST NOT be 1. Each draw_bitmap() queues CASET + params, RASET + params,
    // RAMWR and then the colour data -- several transactions, not one. A depth
    // of 1 makes draw_bitmap() block against itself.
    size_t transQueueDepth{10};

    // 240 * 40 * 2 = 19 200 bytes per buffer, 38.4 KB for the pair. LVGL wants
    // at least 1/10 of the screen (~13.4 KB); with no PSRAM to fall back on
    // there is no reason to spend more.
    uint16_t flushBufferLines{40};

    // Two independent colour bugs, two fields. Getting rgb_ele_order wrong
    // swaps red and blue; getting data_endian wrong turns the image into noise.
    //
    // LCD_RGB_DATA_ENDIAN_LITTLE is the important one: esp_lcd folds it into the
    // ST7789 RAMCTRL register, so the panel reads LVGL's native little-endian
    // RGB565 directly and NOTHING has to byte-swap 134 KB per frame.
    lcd_rgb_element_order_t rgbOrder{LCD_RGB_ELEMENT_ORDER_RGB};
    lcd_rgb_data_endian_t   dataEndian{LCD_RGB_DATA_ENDIAN_LITTLE};

    bool invertColors{true};  // IPS panel

    // ---- backlight ----
    //
    // clockSource is a GLOBAL property of LEDC on the ESP32-S3: every timer must
    // agree on it (driver/ledc.h, ledc_timer_config_t::clk_cfg). Changing it
    // here changes it for components/buzzer too, and ledc_timer_config() returns
    // ESP_FAIL outright when two timers disagree.
    //
    // XTAL because it is the only accurate source that survives light sleep.
    // APB cannot stay powered during sleep, so KEEP_ALIVE would silently do
    // nothing; RC_FAST survives but is +/-7%, wide enough to detune the buzzer.
    ledc_timer_t      backlightTimer{LEDC_TIMER_1};
    ledc_channel_t    backlightChannel{LEDC_CHANNEL_1};
    ledc_timer_bit_t  backlightResolution{LEDC_TIMER_10_BIT};
    uint32_t          backlightFreqHz{20'000};   // above hearing; lower whines
    ledc_clk_cfg_t    clockSource{LEDC_USE_XTAL_CLK};

    // KEEP_ALIVE, the opposite of the buzzer's choice, and for the opposite
    // reason. The UI task blocks on its queue WHILE THE USER IS LOOKING at the
    // screen, so the system may light-sleep with the backlight lit; the default
    // NO_ALIVE_NO_PD would stop the PWM and the screen would go dark mid-glance.
    ledc_sleep_mode_t backlightSleepMode{LEDC_SLEEP_MODE_KEEP_ALIVE};

    bool     gammaBrightness{true};
    uint8_t  defaultBrightness{80};
    uint32_t defaultFadeMs{150};

    // Never infinite. A dead panel must not trap the UI task forever, or the
    // watch stays lit until the battery is flat. Section 7.3.
    uint32_t transferTimeoutMs{200};

    // Log every power Step together with the duty READ BACK OUT OF LEDC.
    //
    // Worth the noise during bring-up, and the reason is a bug this flag would
    // have caught on sight: the policy believed the backlight was lit while the
    // hardware sat at duty 0, so the panel stayed dark through four test
    // patterns. Printing what we asked for would have shown nothing wrong --
    // only the readback separates "we set it" from "it is set". Same trick as
    // buzzerSelfTest() in main.cpp.
    bool logSteps{true};
};

// Returns "was a higher priority task woken", i.e. the portYIELD_FROM_ISR flag.
// Dropping that return value is what makes lv_display_flush_ready() wait for the
// next tick instead of switching immediately.
//
// RUNS IN ISR CONTEXT. No logging, no allocation, no blocking, nothing without a
// FromISR suffix.
using TransferDoneFn = bool (*)(void* ctx);

class DisplayManager {
public:
    DisplayManager() = default;
    ~DisplayManager();

    DisplayManager(const DisplayManager&)            = delete;
    DisplayManager& operator=(const DisplayManager&) = delete;

    // Rolls back everything it built if any step fails, backlight first.
    esp_err_t init(const Wiring& wiring, const Config& config);

    // Safe after a failed init() and safe to call twice.
    void deinit();

    bool isInitialized() const { return initialized_; }

    // ---- flush buffers ----
    //
    // The component allocates them, not the caller. "DMA-capable internal RAM"
    // cannot be enforced if the ui layer is free to malloc its own: an ordinary
    // heap buffer works for a while and then fails in a way nobody can
    // reproduce.
    void*  allocFlushBuffer();
    void   freeFlushBuffer(void* buffer);
    size_t flushBufferBytes() const;

    // ---- drawing ----
    //
    // Does NOT copy the pixels. The buffer must stay alive and untouched until
    // TransferDoneFn fires.
    //
    // Returns ESP_ERR_INVALID_STATE when the policy is not accepting flushes --
    // never silently discards, or a lost frame would look like a rendering bug.
    esp_err_t drawRgb565(const Area& area, const void* dmaBuffer);

    // Blocks until every queued colour transfer has completed. ESP_ERR_TIMEOUT
    // if it does not happen in time; the caller decides what to do, and the
    // sleep path deliberately carries on regardless.
    esp_err_t waitIdle(uint32_t timeoutMs);

    bool     isTransferPending() const { return pending_.load() > 0; }
    uint32_t pendingTransfers() const { return static_cast<uint32_t>(pending_.load()); }

    void setTransferDoneCallback(TransferDoneFn fn, void* ctx);

    // ---- power ----
    esp_err_t setBrightness(uint8_t percent, uint32_t fadeMs);
    esp_err_t enterSleep();
    esp_err_t exitSleep();
    esp_err_t setDimmed(bool dimmed);

    // Latch GPIO15 so it keeps its off level through DEEP sleep. Call after
    // enterSleep(); returns ESP_ERR_INVALID_STATE while a transfer is in
    // flight. The caller still has to invoke gpio_deep_sleep_hold_en() once.
    esp_err_t holdBacklightThroughDeepSleep();

    // Dumps policy state next to what the peripherals actually report, so a
    // divergence between the two shows up in the log rather than in somebody
    // having to describe what the screen is doing.
    void logDiagnostics(const char* where) const;

    // Straight out of the LEDC registers, not the value we last requested.
    uint32_t backlightDutyReadback() const;

    State           state() const { return policy_.state(); }
    uint8_t         brightness() const { return policy_.brightness(); }
    bool            acceptsFlush() const { return policy_.acceptsFlush(); }
    const Geometry& geometry() const { return config_.geometry; }
    const Config&   config() const { return config_; }

private:
    static constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;

    // The ST7789 driver already waits 100 ms inside disp_sleep(); the datasheet
    // asks for 120 before the next command, so this is the remainder.
    static constexpr uint32_t kExtraSleepDelayMs = 20;

    static bool onColorTransDone(esp_lcd_panel_io_handle_t io,
                                 esp_lcd_panel_io_event_data_t* edata,
                                 void* ctx);

    esp_err_t initSpi();
    esp_err_t initPanel();
    esp_err_t initBacklight();

    // Runs the PowerPolicy sequence to completion. Returns the first error but
    // always finishes the sequence -- stopping halfway would leave the panel in
    // a state neither the policy nor the hardware agrees on.
    esp_err_t runPolicy();
    esp_err_t applyStep(Step step);
    esp_err_t clearScreen(uint16_t colourRgb565);
    static const char* stepName(Step step);
    esp_err_t applyBacklight(uint8_t percent, uint32_t fadeMs);

    Wiring config_wiring_{};
    Config config_{};

    esp_lcd_panel_io_handle_t io_{nullptr};
    esp_lcd_panel_handle_t    panel_{nullptr};
    SemaphoreHandle_t         transferDone_{nullptr};

    PowerPolicy policy_{};

    TransferDoneFn userCallback_{nullptr};
    void*          userCallbackCtx_{nullptr};

    std::atomic<int> pending_{0};

    bool initialized_{false};
    bool spiBusOwned_{false};
    bool backlightReady_{false};
    bool fadeServiceOwned_{false};
    bool redrawRequested_{false};

public:
    // True when the policy asked for a full redraw during the last wake and the
    // ui layer has not acted on it yet. Cleared by takeRedrawRequest().
    bool takeRedrawRequest();
};

}  // namespace display
