#include "display_manager.hpp"

#include <cinttypes>
#include <cstring>

#include "esp_attr.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

namespace display {
namespace {
constexpr const char* TAG = "display";
}  // namespace

DisplayManager::~DisplayManager()
{
    deinit();
}

// ------------------------------------------------------------------------ init

esp_err_t DisplayManager::init(const Wiring& wiring, const Config& config)
{
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    config_wiring_ = wiring;
    config_        = config;

    // Caught here rather than trusted to whoever edits Config next. esp_lcd adds
    // the gap to the coordinates while mirror writes MADCTL; the two only agree
    // when the visible window sits centred in GRAM. See display.md section 9.3.
    if (!config_.geometry.isCentredInGram()) {
        ESP_LOGE(TAG,
                 "panel %ux%u voi gap %u/%u khong nam giua GRAM %ux%u -- doi "
                 "mirror se lam troi anh. Xem display.md muc 9.3.",
                 config_.geometry.width, config_.geometry.height,
                 config_.geometry.xGap, config_.geometry.yGap,
                 config_.geometry.gramWidth, config_.geometry.gramHeight);
        return ESP_ERR_INVALID_ARG;
    }

    if (config_.transQueueDepth < 2) {
        ESP_LOGE(TAG, "transQueueDepth=%u qua nho: mot draw_bitmap() phat nhieu "
                      "giao dich (CASET/RASET/RAMWR/data).",
                 static_cast<unsigned>(config_.transQueueDepth));
        return ESP_ERR_INVALID_ARG;
    }

    transferDone_ = xSemaphoreCreateBinary();
    if (transferDone_ == nullptr) {
        deinit();
        return ESP_ERR_NO_MEM;
    }

    // BACKLIGHT FIRST, before SPI and before the panel exists.
    //
    // GPIO15 floats out of reset, and letting it float until after the panel is
    // on shows a white flash at every boot. ledc_channel_config() writes duty 0
    // and only then attaches the pad (ledc.c:921-929), so the pin is driven low
    // the instant LEDC takes it -- the same guarantee a bare gpio_set_level()
    // gives, held by the peripheral that keeps it rather than by one that has to
    // hand it over.
    //
    // An earlier version did call gpio_config() here as well. That was worse
    // than redundant: gpio_config() reserves the pad (gpio.c:390), so LEDC then
    // logged "GPIO 15 is not usable, maybe conflict with others" while claiming
    // it anyway -- a warning pointing at a conflict that did not exist.
    esp_err_t err = initBacklight();
    if (err != ESP_OK) {
        deinit();
        return err;
    }

    err = initSpi();
    if (err != ESP_OK) {
        deinit();
        return err;
    }

    err = initPanel();
    if (err != ESP_OK) {
        deinit();
        return err;
    }

    initialized_ = true;

    // The panel is on but the backlight is still at duty 0, and that is exactly
    // what onInitialized() records. target_ is AWAKE, so the policy now owes a
    // APPLY_BRIGHTNESS -- runPolicy() below is what pays it.
    policy_.onInitialized(config_.defaultBrightness);

    ESP_LOGI(TAG, "san sang: %ux%u, SPI %" PRIu32 " Hz, buffer %u dong (%u byte)",
             config_.geometry.width, config_.geometry.height, config_.spiClockHz,
             config_.flushBufferLines,
             static_cast<unsigned>(flushBufferBytes()));

    // Paint black BEFORE the light comes up. GRAM is undefined at power-on, so
    // fading up first would reveal a frame of noise -- the same rule the wake
    // path follows, applied to boot.
    const esp_err_t cerr = clearScreen(0x0000);
    if (cerr != ESP_OK) {
        ESP_LOGW(TAG, "khong xoa duoc man hinh truoc khi bat den: %s",
                 esp_err_to_name(cerr));
    }

    err = runPolicy();
    logDiagnostics("sau init");
    return err;
}

// Paints the whole panel one band at a time through a temporary buffer.
esp_err_t DisplayManager::clearScreen(uint16_t colourRgb565)
{
    auto* buf = static_cast<uint16_t*>(allocFlushBuffer());
    if (buf == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    const size_t words = flushBufferBytes() / sizeof(uint16_t);
    for (size_t i = 0; i < words; ++i) {
        buf[i] = colourRgb565;
    }

    esp_err_t err = ESP_OK;
    for (uint16_t y0 = 0; y0 < config_.geometry.height; y0 += config_.flushBufferLines) {
        uint16_t y1 = static_cast<uint16_t>(y0 + config_.flushBufferLines);
        if (y1 > config_.geometry.height) {
            y1 = config_.geometry.height;
        }
        err = drawRgb565(Area{0, y0, config_.geometry.width, y1}, buf);
        if (err != ESP_OK) {
            break;
        }
        err = waitIdle(config_.transferTimeoutMs);
        if (err != ESP_OK) {
            break;
        }
    }

    freeFlushBuffer(buf);
    return err;
}

esp_err_t DisplayManager::initSpi()
{
    spi_bus_config_t busCfg{};
    busCfg.sclk_io_num     = config_wiring_.sclk;
    busCfg.mosi_io_num     = config_wiring_.mosi;
    busCfg.miso_io_num     = -1;  // not wired on this board
    busCfg.quadwp_io_num   = -1;
    busCfg.quadhd_io_num   = -1;
    // WITHOUT THIS the driver defaults to 4092 bytes and every flush fails.
    // 19 200 > 4092, and the resulting error looks like a wiring fault.
    busCfg.max_transfer_sz = static_cast<int>(flushBufferBytes());

    esp_err_t err = spi_bus_initialize(config_wiring_.spiHost, &busCfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize() that bai: %s", esp_err_to_name(err));
        return err;
    }
    spiBusOwned_ = true;

    esp_lcd_panel_io_spi_config_t ioCfg{};
    ioCfg.cs_gpio_num       = config_wiring_.cs;
    ioCfg.dc_gpio_num       = config_wiring_.dc;
    ioCfg.spi_mode          = 0;
    ioCfg.pclk_hz           = config_.spiClockHz;
    ioCfg.trans_queue_depth = config_.transQueueDepth;
    ioCfg.lcd_cmd_bits      = 8;
    ioCfg.lcd_param_bits    = 8;
    ioCfg.on_color_trans_done = &DisplayManager::onColorTransDone;
    ioCfg.user_ctx            = this;

    // esp_lcd_spi_bus_handle_t is a typedef for int, not a pointer.
    err = esp_lcd_new_panel_io_spi(
        static_cast<esp_lcd_spi_bus_handle_t>(config_wiring_.spiHost), &ioCfg, &io_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_io_spi() that bai: %s", esp_err_to_name(err));
        io_ = nullptr;
        return err;
    }
    return ESP_OK;
}

esp_err_t DisplayManager::initPanel()
{
    esp_lcd_panel_dev_config_t panelCfg{};
    panelCfg.reset_gpio_num = config_wiring_.reset;
    panelCfg.rgb_ele_order  = config_.rgbOrder;
    panelCfg.data_endian    = config_.dataEndian;
    panelCfg.bits_per_pixel = 16;

    esp_err_t err = esp_lcd_new_panel_st7789(io_, &panelCfg, &panel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_st7789() that bai: %s", esp_err_to_name(err));
        panel_ = nullptr;
        return err;
    }

    // reset() and init() together cost over 100 ms: the ST7789 driver sends
    // SLPOUT and then waits 100 ms inside init().
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel_), TAG, "panel reset that bai");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel_), TAG, "panel init that bai");

    // Order matters: mirror and swap write MADCTL, and set_gap is applied to
    // coordinates afterwards.
    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_mirror(panel_, config_.geometry.mirrorX, config_.geometry.mirrorY),
        TAG, "mirror that bai");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(panel_, config_.geometry.swapXy),
                        TAG, "swap_xy that bai");
    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_set_gap(panel_, config_.geometry.xGap, config_.geometry.yGap),
        TAG, "set_gap that bai");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(panel_, config_.invertColors),
                        TAG, "invert_color that bai");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel_, true),
                        TAG, "disp_on that bai");

    return ESP_OK;
}

esp_err_t DisplayManager::initBacklight()
{
    ledc_timer_config_t timerCfg{};
    timerCfg.speed_mode      = kSpeedMode;
    timerCfg.timer_num       = config_.backlightTimer;
    timerCfg.duty_resolution = config_.backlightResolution;
    timerCfg.freq_hz         = config_.backlightFreqHz;
    timerCfg.clk_cfg         = config_.clockSource;

    esp_err_t err = ledc_timer_config(&timerCfg);
    if (err != ESP_OK) {
        // The single most likely failure here, and the message it produces on
        // its own points nowhere near the real cause.
        ESP_LOGE(TAG, "ledc_timer_config() that bai: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "  Neu la ESP_FAIL: clock source cua LEDC la TOAN CUC tren "
                      "ESP32-S3. Kiem tra BuzzerManager::Config::clockSource co "
                      "dung LEDC_USE_XTAL_CLK khong. Xem display.md muc 10.2.");
        return err;
    }

    ledc_channel_config_t chCfg{};
    chCfg.gpio_num   = static_cast<int>(config_wiring_.backlight);
    chCfg.speed_mode = kSpeedMode;
    chCfg.channel    = config_.backlightChannel;
    chCfg.timer_sel  = config_.backlightTimer;
    chCfg.duty       = 0;      // stay dark until the first fade
    chCfg.hpoint     = 0;
    chCfg.sleep_mode = config_.backlightSleepMode;

    err = ledc_channel_config(&chCfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config() that bai: %s", esp_err_to_name(err));
        return err;
    }
    backlightReady_ = true;

    // Global to the whole LEDC peripheral, not per channel. This component owns
    // it because nothing else currently fades; if the buzzer ever wants fades,
    // ownership has to move up to the application rather than be installed
    // twice.
    err = ledc_fade_func_install(0);
    if (err == ESP_OK) {
        fadeServiceOwned_ = true;
    } else if (err == ESP_ERR_INVALID_STATE) {
        // Someone else already installed it; fades still work, we just must not
        // uninstall it in deinit().
        ESP_LOGW(TAG, "ledc fade service da duoc cai boi mot component khac");
    } else {
        ESP_LOGE(TAG, "ledc_fade_func_install() that bai: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

void DisplayManager::deinit()
{
    // Light out first, whatever else happens.
    if (backlightReady_) {
        ledc_stop(kSpeedMode, config_.backlightChannel, 0);
    }
    if (initialized_) {
        waitIdle(config_.transferTimeoutMs);
    }

    if (fadeServiceOwned_) {
        ledc_fade_func_uninstall();
        fadeServiceOwned_ = false;
    }
    backlightReady_ = false;

    if (panel_ != nullptr) {
        esp_lcd_panel_del(panel_);
        panel_ = nullptr;
    }
    if (io_ != nullptr) {
        esp_lcd_panel_io_del(io_);
        io_ = nullptr;
    }
    if (spiBusOwned_) {
        spi_bus_free(config_wiring_.spiHost);
        spiBusOwned_ = false;
    }
    if (transferDone_ != nullptr) {
        vSemaphoreDelete(transferDone_);
        transferDone_ = nullptr;
    }

    pending_.store(0);
    policy_.reset();
    initialized_ = false;
}

// --------------------------------------------------------------- flush buffers

size_t DisplayManager::flushBufferBytes() const
{
    return static_cast<size_t>(config_.geometry.width) *
           static_cast<size_t>(config_.flushBufferLines) * 2u;
}

void* DisplayManager::allocFlushBuffer()
{
    void* p = heap_caps_malloc(flushBufferBytes(), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (p == nullptr) {
        ESP_LOGE(TAG, "khong cap phat duoc %u byte DMA internal RAM",
                 static_cast<unsigned>(flushBufferBytes()));
    }
    return p;
}

void DisplayManager::freeFlushBuffer(void* buffer)
{
    heap_caps_free(buffer);
}

// --------------------------------------------------------------------- drawing

bool IRAM_ATTR DisplayManager::onColorTransDone(esp_lcd_panel_io_handle_t,
                                                esp_lcd_panel_io_event_data_t*,
                                                void* ctx)
{
    auto* self = static_cast<DisplayManager*>(ctx);
    self->pending_.fetch_sub(1, std::memory_order_release);

    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(self->transferDone_, &woken);

    bool userWoken = false;
    if (self->userCallback_ != nullptr) {
        userWoken = self->userCallback_(self->userCallbackCtx_);
    }

    // Returning this is what lets the scheduler switch immediately instead of
    // waiting for the next tick. Dropping it costs a full tick per flush.
    return (woken == pdTRUE) || userWoken;
}

void DisplayManager::setTransferDoneCallback(TransferDoneFn fn, void* ctx)
{
    userCallbackCtx_ = ctx;
    userCallback_    = fn;
}

esp_err_t DisplayManager::drawRgb565(const Area& area, const void* dmaBuffer)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dmaBuffer == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!isValid(area, config_.geometry)) {
        ESP_LOGW(TAG, "vung khong hop le: (%u,%u)-(%u,%u)",
                 area.x0, area.y0, area.x1, area.y1);
        return ESP_ERR_INVALID_ARG;
    }
    // Never silently dropped: a discarded frame looks exactly like a rendering
    // bug from the ui layer's point of view.
    if (!policy_.acceptsFlush()) {
        return ESP_ERR_INVALID_STATE;
    }

    pending_.fetch_add(1, std::memory_order_acquire);
    const esp_err_t err = esp_lcd_panel_draw_bitmap(panel_, area.x0, area.y0,
                                                    area.x1, area.y1, dmaBuffer);
    if (err != ESP_OK) {
        pending_.fetch_sub(1, std::memory_order_release);
        ESP_LOGE(TAG, "draw_bitmap() that bai: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t DisplayManager::waitIdle(uint32_t timeoutMs)
{
    if (transferDone_ == nullptr) {
        return ESP_OK;
    }

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);

    while (pending_.load(std::memory_order_acquire) > 0) {
        const TickType_t now = xTaskGetTickCount();
        if (now >= deadline) {
            ESP_LOGW(TAG, "waitIdle() het gio, con %d giao dich dang bay",
                     pending_.load());
            return ESP_ERR_TIMEOUT;
        }
        // Polling the flag would burn the CPU of every lower priority task; wait
        // on the semaphore the ISR gives instead.
        xSemaphoreTake(transferDone_, deadline - now);
    }
    return ESP_OK;
}

bool DisplayManager::takeRedrawRequest()
{
    const bool r     = redrawRequested_;
    redrawRequested_ = false;
    return r;
}

// ----------------------------------------------------------------------- power

esp_err_t DisplayManager::applyBacklight(uint8_t percent, uint32_t fadeMs)
{
    if (!backlightReady_) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint32_t duty = dutyForPercent(
        percent, static_cast<uint8_t>(config_.backlightResolution),
        config_.gammaBrightness);

    if (fadeMs == 0 || !fadeServiceOwned_) {
        ESP_RETURN_ON_ERROR(ledc_set_duty(kSpeedMode, config_.backlightChannel, duty),
                            TAG, "ledc_set_duty that bai");
        return ledc_update_duty(kSpeedMode, config_.backlightChannel);
    }

    // WAIT_DONE, not NO_WAIT: the sleep sequence must not send DISPOFF while the
    // light is still on its way down. Blocking the UI task here is intended.
    return ledc_set_fade_time_and_start(kSpeedMode, config_.backlightChannel, duty,
                                        fadeMs, LEDC_FADE_WAIT_DONE);
}

esp_err_t DisplayManager::applyStep(Step step)
{
    switch (step) {
    case Step::BACKLIGHT_OFF:
        return applyBacklight(0, config_.defaultFadeMs);

    case Step::APPLY_BRIGHTNESS:
        return applyBacklight(policy_.brightness(), config_.defaultFadeMs);

    case Step::WAIT_TRANSFER:
        return waitIdle(config_.transferTimeoutMs);

    case Step::PANEL_OFF:
        return esp_lcd_panel_disp_on_off(panel_, false);

    case Step::PANEL_ON:
        return esp_lcd_panel_disp_on_off(panel_, true);

    case Step::PANEL_SLEEP_IN:
        return esp_lcd_panel_disp_sleep(panel_, true);

    case Step::PANEL_SLEEP_OUT: {
        // The driver already waits 100 ms; the ST7789V2 datasheet asks for 120
        // before the next command.
        const esp_err_t err = esp_lcd_panel_disp_sleep(panel_, false);
        vTaskDelay(pdMS_TO_TICKS(kExtraSleepDelayMs));
        return err;
    }

    case Step::REQUEST_REDRAW:
        // GRAM contents are undefined across SLPIN/SLPOUT. The ui layer must
        // repaint before the light comes up, so this only raises a flag.
        redrawRequested_ = true;
        return ESP_OK;

    case Step::NONE:
        return ESP_OK;
    }
    return ESP_OK;
}

const char* DisplayManager::stepName(Step step)
{
    switch (step) {
    case Step::NONE:                return "NONE";
    case Step::BACKLIGHT_OFF:   return "BACKLIGHT_OFF";
    case Step::APPLY_BRIGHTNESS: return "APPLY_BRIGHTNESS";
    case Step::WAIT_TRANSFER:       return "WAIT_TRANSFER";
    case Step::PANEL_OFF:           return "PANEL_OFF";
    case Step::PANEL_ON:            return "PANEL_ON";
    case Step::PANEL_SLEEP_IN:      return "PANEL_SLEEP_IN";
    case Step::PANEL_SLEEP_OUT:     return "PANEL_SLEEP_OUT";
    case Step::REQUEST_REDRAW:      return "REQUEST_REDRAW";
    }
    return "?";
}

uint32_t DisplayManager::backlightDutyReadback() const
{
    if (!backlightReady_) {
        return 0;
    }
    return ledc_get_duty(kSpeedMode, config_.backlightChannel);
}

void DisplayManager::logDiagnostics(const char* where) const
{
    static const char* kStateName[] = {"UNINIT", "AWAKE", "DIMMED", "PANEL_SLEEP"};

    const uint32_t want = dutyForPercent(
        policy_.brightness(), static_cast<uint8_t>(config_.backlightResolution),
        config_.gammaBrightness);
    const uint32_t got  = backlightDutyReadback();
    const uint32_t full = 1u << static_cast<uint32_t>(config_.backlightResolution);

    ESP_LOGI(TAG, "[%s] trang thai=%s sang=%u%% duty=%" PRIu32 "/%" PRIu32
                  " (mong %" PRIu32 ") flush=%s dangbay=%" PRIu32 " can-ve-lai=%d",
             where, kStateName[static_cast<int>(policy_.state())],
             policy_.brightness(), got, full, want,
             policy_.acceptsFlush() ? "cho" : "chan",
             pendingTransfers(), static_cast<int>(redrawRequested_));

    // The check that would have caught the dark-screen bug on its own: the
    // policy claiming AWAKE while LEDC sits at 0 is a contradiction no amount of
    // staring at the panel can diagnose faster than one line of log.
    if (policy_.state() == State::AWAKE && policy_.brightness() > 0 && got == 0) {
        ESP_LOGE(TAG, "  MAU THUAN: policy noi AWAKE %u%% ma LEDC dang o duty 0 "
                      "-> den nen TAT. Mo hinh trang thai lech voi phan cung.",
                 policy_.brightness());
    }
    if (policy_.state() == State::PANEL_SLEEP && got != 0) {
        ESP_LOGE(TAG, "  MAU THUAN: panel dang ngu ma den nen van sang (duty %"
                      PRIu32 ")", got);
    }
}

esp_err_t DisplayManager::runPolicy()
{
    esp_err_t first = ESP_OK;

    // Bounded: the policy has at most a handful of steps, and a runaway loop
    // here would hang the UI task.
    for (int guard = 0; guard < 16; ++guard) {
        const Step step = policy_.nextStep();
        if (step == Step::NONE) {
            return first;
        }

        const esp_err_t err = applyStep(step);
        if (err != ESP_OK && first == ESP_OK) {
            first = err;
        }

        if (config_.logSteps) {
            // The duty is READ BACK from LEDC, not echoed from what we asked
            // for. That difference is the whole point: it is what turns "the
            // screen is dark" from something you have to describe into
            // something the log already says.
            ESP_LOGI(TAG, "  step %-15s %s  duty=%" PRIu32,
                     stepName(step),
                     err == ESP_OK ? "ok " : esp_err_to_name(err),
                     backlightDutyReadback());
        }

        // Advances even on failure. A timed-out WAIT_TRANSFER must not strand
        // the sequence: leaving the screen lit is worse than sleeping with a
        // transfer possibly in flight. Section 7.3.
        policy_.stepDone();
    }

    ESP_LOGE(TAG, "runPolicy() khong hoi tu -- day la loi logic, khong phai phan cung");
    return first == ESP_OK ? ESP_ERR_INVALID_STATE : first;
}

esp_err_t DisplayManager::setBrightness(uint8_t percent, uint32_t fadeMs)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint32_t saved = config_.defaultFadeMs;
    config_.defaultFadeMs = fadeMs;
    policy_.request(Request::SET_BRIGHTNESS, percent);
    const esp_err_t err = runPolicy();
    config_.defaultFadeMs = saved;
    return err;
}

esp_err_t DisplayManager::enterSleep()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    policy_.request(Request::SLEEP);
    return runPolicy();
}

esp_err_t DisplayManager::exitSleep()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    policy_.request(Request::WAKE);
    return runPolicy();
}

esp_err_t DisplayManager::setDimmed(bool dimmed)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    policy_.request(dimmed ? Request::DIM : Request::WAKE);
    return runPolicy();
}

esp_err_t DisplayManager::holdBacklightThroughDeepSleep()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (isTransferPending()) {
        return ESP_ERR_INVALID_STATE;
    }

    // Same failure mode as the buzzer pin: a floating pad on a transistor stage
    // can leave the backlight partly on -- invisible, but it drains the battery
    // for hours. The caller still owes a gpio_deep_sleep_hold_en().
    ESP_RETURN_ON_ERROR(ledc_stop(kSpeedMode, config_.backlightChannel, 0), TAG,
                        "ledc_stop that bai");
    return gpio_hold_en(config_wiring_.backlight);
}

}  // namespace display
