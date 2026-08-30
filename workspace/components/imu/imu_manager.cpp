#include "imu_manager.hpp"

#include <cinttypes>
#include <atomic>
#include <cmath>
#include <new>

#include "esp_log.h"
#include "esp_timer.h"

namespace imu {

namespace {

constexpr const char* TAG = "imu";

// Counts ISR entries. Deliberately a plain file-scope atomic rather than a
// member: the handler is given a TaskHandle_t, not `this`, precisely so it
// dereferences no object. One IMU per system, so this is enough -- and it
// is the only way to tell "the pin never pulses" from "the pin pulses too
// narrowly for gpio_get_level() to ever catch it".
std::atomic<uint32_t> g_isrCount{0};

inline uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

void delayMs(uint32_t ms)
{
    // Clamp the TICKS, not the milliseconds. At CONFIG_FREERTOS_HZ=100 one
    // tick is 10 ms, so pdMS_TO_TICKS(1) and pdMS_TO_TICKS(2) are both ZERO
    // and vTaskDelay(0) does not delay at all -- it just yields.
    //
    // Guarding "ms == 0" looks like the same thing and is not. That version
    // made the CTRL9 poll spin through all 100 attempts in ~25 ms instead of
    // 100 ms (so the first WoM exit timed out), and made a 200 ms diagnostic
    // window sample for 25 ms (so it under-counted by 6x and I nearly drew
    // the wrong conclusion from it).
    //
    // button.hpp devotes a whole section to this trap. It caught me anyway.
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ticks == 0) {
        ticks = 1;
    }
    vTaskDelay(ticks);
}

}  // namespace

ImuManager::~ImuManager()
{
    stop();
    if (dev_ != nullptr) {
        dev_->~Qmi8658();
        dev_ = nullptr;
    }
}

// ------------------------------------------------------------------------ init

esp_err_t ImuManager::init(const Config& config)
{
    if (started_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (config.bus == nullptr) {
        ESP_LOGE(TAG, "Config::bus rong -- app phai tao i2c::Device truoc");
        return ESP_ERR_INVALID_ARG;
    }
    if (!GPIO_IS_VALID_GPIO(config.intPin)) {
        ESP_LOGE(TAG, "GPIO%d khong hop le", static_cast<int>(config.intPin));
        return ESP_ERR_INVALID_ARG;
    }

    config_ = config;

    if (dev_ != nullptr) {
        dev_->~Qmi8658();
    }
    dev_ = new (devStorage_) Qmi8658(*config_.bus, delayMs);
    dev_->setRanges(config_.accelRange, config_.gyroRange);
    dev_->setWomConfig(config_.wom);

    // ---- section 13, in this order and for the reasons given there ----

    // 3. soft reset, then confirm. 0x4D is only valid right now.
    if (!dev_->beginReset()) {
        ESP_LOGE(TAG, "khong ghi duoc lenh reset -- kiem tra day I2C");
        return ESP_ERR_NOT_FOUND;
    }
    delayMs(20);  // datasheet: reset takes at most 15 ms
    if (!dev_->resetSucceeded()) {
        ESP_LOGW(TAG, "0x4D khong tra ve 0x80 sau reset");
    }

    // 4. address auto-increment. Without this every burst read returns the same
    //    register over and over -- data that looks plausible and is nonsense.
    if (!dev_->setAddressAutoIncrement(true)) {
        ESP_LOGE(TAG, "khong dat duoc CTRL1.ADDR_AI");
        return ESP_ERR_NOT_FOUND;
    }

    // 2. Identity, staged. Deliberately AFTER ADDR_AI so it can also report
    //    whether that step actually took effect -- and say WHICH of the
    //    three possible faults it hit, with the bytes it saw. "Probe failed"
    //    on its own names three different problems and fixes none of them.
    Qmi8658::IdentityReport id{};
    const bool identified = dev_->identify(id);

    if (!id.read) {
        ESP_LOGE(TAG, "0x6B tra loi luc quet bus nhung khong doc duoc thanh ghi "
                      "-- kiem tra dien tro keo hoac nhieu tren duong I2C");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "WHO_AM_I=0x%02X  REVISION_ID=0x%02X  burst={0x%02X,0x%02X}",
             id.whoAmI, id.revision, id.burst[0], id.burst[1]);

    if (!id.isQmi8658) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X, mong doi 0x05 -- khong phai QMI8658C",
                 id.whoAmI);
        return ESP_ERR_NOT_FOUND;
    }

    if (!identified) {
        // The chip answers single-byte reads but no burst convention works.
        // Every multi-byte read in this driver would return the same register
        // repeated, so refusing here beats shipping plausible nonsense.
        ESP_LOGE(TAG, "dung la QMI8658C nhung KHONG doc burst duoc: doc 2 byte tu "
                      "0x00 ra {0x%02X,0x%02X}, mong {0x%02X,0x%02X}. Ca hai quy uoc "
                      "auto-increment (CTRL1.bit6 va reg|0x80) deu khong an.",
                 id.burst[0], id.burst[1], id.whoAmI, id.revision);
        return ESP_ERR_NOT_FOUND;
    }

    if (id.needsHighBitBurst) {
        // Rev 0.6 section 12.2 was right about this part after all.
        ESP_LOGW(TAG, "part nay chi auto-increment khi bit7 cua dia chi thanh ghi "
                      "duoc bat -- driver da tu chuyen sang quy uoc do");
    }

    if (id.revision != reg::REVISION_ID_VALUE) {
        // Informational only. A different revision byte is not a fault, and
        // an over-specified check would have condemned a working part.
        ESP_LOGW(TAG, "REVISION_ID=0x%02X (thuong la 0x68) -- van chay binh thuong",
                 id.revision);
    }

    // 5. INT1 is not routed on this board, so route the CTRL9 handshake to a
    //    register and every activity interrupt to INT2. Skip this and the first
    //    CTRL9 command waits forever on a pin that does not exist.
    if (!dev_->setCtrl9Handshake(Ctrl9Handshake::StatusRegister) ||
        !dev_->setActivityIntPin(IntPin::Int2)) {
        ESP_LOGE(TAG, "khong cau hinh duoc CTRL8");
        return ESP_ERR_NOT_FOUND;
    }

    // 5b. CTRL1 bit 4. Undocumented in Rev A ("Reserved"), used by SensorLib
    //     as the INT2 output enable. Without it this board produced exactly
    //     zero interrupts on GPIO38 -- the ODR self-test counted 0.0 Hz while
    //     every register read back correctly.
    if (config_.enableInt2OutputBit) {
        uint8_t ctrl1 = 0;
        if (dev_->readRegister(reg::CTRL1, ctrl1)) {
            const uint8_t want =
                static_cast<uint8_t>(ctrl1 | reg::ctrl1::INT2_ENABLE_UNDOCUMENTED);
            if (want != ctrl1 && !dev_->writeRegister(reg::CTRL1, want)) {
                ESP_LOGW(TAG, "khong dat duoc CTRL1.bit4 (bat xuat INT2)");
            }
        }
    }

    // 10. the GPIO. ANYEDGE, not POSEDGE: a WoM event TOGGLES the pin rather
    //     than pulsing it, so half the events would be invisible on one edge.
    gpio_config_t io{};
    io.pin_bit_mask = 1ULL << static_cast<uint32_t>(config_.intPin);
    io.mode         = GPIO_MODE_INPUT;
    // Pull-up ON. If the QMI8658C drives INT2 push-pull this changes nothing;
    // if it is open-drain -- which the datasheet never states either way --
    // then without a pull-up the pad can only ever be pulled LOW and never
    // returns high, so no edge is ever produced. That matches exactly what
    // this board does: the chip asserts INT2 internally (STATUSINT.bit0) while
    // the pad never moves. Cheap to try, and harmless if the guess is wrong.
    io.pull_up_en   = config_.intPullUp ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_ANYEDGE;

    const esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(GPIO%d): %s", static_cast<int>(config_.intPin),
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "khoi tao xong tren GPIO%d", static_cast<int>(config_.intPin));
    return ESP_OK;
}

// ------------------------------------------------------------------ start/stop

esp_err_t ImuManager::start()
{
    if (started_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (dev_ == nullptr) {
        ESP_LOGE(TAG, "phai goi init() truoc start()");
        return ESP_ERR_INVALID_STATE;
    }

    stopRequested_.store(false, std::memory_order_release);
    requestedMode_.store(PowerMode::WOM, std::memory_order_release);

    eventQueue_ = xQueueCreate(config_.queueLength, sizeof(ImuEventMsg));
    if (eventQueue_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    stopSemaphore_ = xSemaphoreCreateBinary();
    selfTestDone_  = xSemaphoreCreateBinary();
    if (stopSemaphore_ == nullptr || selfTestDone_ == nullptr) {
        releaseResources();
        return ESP_ERR_NO_MEM;
    }

    // The task must exist before the ISR is registered: the handler argument is
    // copied by value at registration time.
    if (xTaskCreatePinnedToCore(taskFunc, "imu", config_.taskStackSize, this,
                                config_.taskPriority, &taskHandle_,
                                config_.taskCoreId) != pdPASS) {
        taskHandle_ = nullptr;
        releaseResources();
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = ESP_OK;
    if (config_.installIsrService) {
        err = gpio_install_isr_service(0);
        if (err == ESP_ERR_INVALID_STATE) {
            isrServiceOwned_ = false;  // someone got there first: fine
            err              = ESP_OK;
        } else if (err != ESP_OK) {
            teardownTask();
            return err;
        } else {
            isrServiceOwned_ = true;
        }
    }

    err = gpio_isr_handler_add(config_.intPin, isrHandler, taskHandle_);
    if (err != ESP_OK) {
        teardownTask();
        return err;
    }

    started_ = true;
    return ESP_OK;
}

void ImuManager::stop()
{
    if (!started_) {
        return;
    }
    gpio_isr_handler_remove(config_.intPin);
    teardownTask();

    uint32_t settle = 0;
    if (dev_ != nullptr) {
        dev_->applyMode(PowerMode::OFF, settle);  // never leave the gyro running
    }
    started_ = false;
}

bool ImuManager::haltTask()
{
    if (taskHandle_ == nullptr) {
        return true;
    }
    // Flag first, then wake. The other order races: the task wakes, sees a
    // clear flag, services one interrupt and sleeps again -- with the
    // notification already consumed.
    stopRequested_.store(true, std::memory_order_release);
    xTaskNotifyGive(taskHandle_);

    return stopSemaphore_ != nullptr &&
           xSemaphoreTake(stopSemaphore_, pdMS_TO_TICKS(kStopTimeoutMs)) == pdTRUE;
}

void ImuManager::teardownTask()
{
    if (haltTask()) {
        releaseResources();
    } else {
        // Same policy as ButtonManager: a bounded, logged leak beats freeing
        // memory a live task still points at.
        ESP_LOGE(TAG, "task khong thoat trong %" PRIu32 " ms: co y ro ri queue",
                 kStopTimeoutMs);
        eventQueue_    = nullptr;
        stopSemaphore_ = nullptr;
    }
    taskHandle_ = nullptr;
}

void ImuManager::releaseResources()
{
    if (eventQueue_ != nullptr) {
        vQueueDelete(eventQueue_);
        eventQueue_ = nullptr;
    }
    if (stopSemaphore_ != nullptr) {
        vSemaphoreDelete(stopSemaphore_);
        stopSemaphore_ = nullptr;
    }
    if (selfTestDone_ != nullptr) {
        vSemaphoreDelete(selfTestDone_);
        selfTestDone_ = nullptr;
    }
}

// ------------------------------------------------------------------ public API

esp_err_t ImuManager::setPowerMode(PowerMode mode)
{
    if (!started_) {
        return ESP_ERR_INVALID_STATE;
    }
    // Record and wake. Touching the chip from the caller's task would break the
    // one-task-one-device rule the whole design rests on.
    requestedMode_.store(mode, std::memory_order_release);
    xTaskNotifyGive(taskHandle_);
    return ESP_OK;
}

float ImuManager::accelOdrHz() const
{
    return dev_ != nullptr ? dev_->accelOdrHz() : 0.0f;
}

bool ImuManager::latest(sensors::Sample& out) const
{
    for (int attempt = 0; attempt < 8; ++attempt) {
        const uint32_t before = snapshotSeq_.load(std::memory_order_acquire);
        if (before & 1u) {
            continue;  // a write is in progress
        }
        out = snapshot_;
        std::atomic_thread_fence(std::memory_order_acquire);
        if (snapshotSeq_.load(std::memory_order_relaxed) == before) {
            return before != 0;  // 0 means nothing has ever been published
        }
    }
    return false;
}

bool ImuManager::waitEvent(ImuEventMsg& out, uint32_t timeoutMs)
{
    if (eventQueue_ == nullptr) {
        return false;
    }
    const TickType_t ticks =
        (timeoutMs == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
    return xQueueReceive(eventQueue_, &out, ticks) == pdTRUE;
}

// -------------------------------------------------------------- ISR and task

void IRAM_ATTR ImuManager::isrHandler(void* arg)
{
    // One pin carries seven different meanings (data ready, FIFO watermark,
    // WoM, pedometer, tap, any/no/significant motion), so the ISR cannot tell
    // them apart and must not try: reading STATUS over I2C is a blocking bus
    // transaction. It just wakes the task.
    g_isrCount.fetch_add(1, std::memory_order_relaxed);

    BaseType_t hpTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(arg), &hpTaskWoken);
    if (hpTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void ImuManager::taskFunc(void* arg)
{
    static_cast<ImuManager*>(arg)->run();
}

bool ImuManager::applyPendingMode()
{
    const PowerMode want = requestedMode_.load(std::memory_order_acquire);
    if (want == appliedMode_) {
        return true;
    }

    uint32_t settleMs = 0;
    if (!dev_->applyMode(want, settleMs)) {
        ESP_LOGE(TAG, "khong chuyen duoc sang che do %d", static_cast<int>(want));
        post(ImuEventMsg::Type::BUS_ERROR, nowMs(), 0, ESP_FAIL);
        return false;
    }

    // The sensor needs its wake-up time plus three filter periods before its
    // output means anything. Waiting here, in the sensor's own task, is why
    // Qmi8658 can stay free of FreeRTOS and just report the number.
    if (settleMs > 0) {
        delayMs(settleMs);
    }

    appliedMode_ = want;
    ESP_LOGI(TAG, "che do %d, accel ODR = %.2f Hz", static_cast<int>(want),
             dev_->accelOdrHz());
    return true;
}

void ImuManager::run()
{
    // init() legitimately configured the chip from app_main, so the device
    // still thinks that task owns it. Take it over here, once, before the
    // first access -- otherwise the perfectly correct configure-then-hand-off
    // sequence trips the check meant to catch real violations.
    if (auto* dev = static_cast<i2c::Device*>(config_.bus)) {
        dev->claimOwnership();
    }

    while (!stopRequested_.load(std::memory_order_acquire)) {
        applyPendingMode();

        // Register work for the self-test happens HERE, on the task that
        // owns the device. Doing it from the caller interleaved two CTRL9
        // handshakes and made both fail.
        if (selfTestRequested_.exchange(false, std::memory_order_acq_rel)) {
            performRegisterSelfTest();
            xSemaphoreGive(selfTestDone_);
        }

        // The wait timeout doubles as a polling fallback.
        //
        // In a sampling mode it is one ODR period, so if DRDY interrupts
        // arrive the notification short-circuits the wait and nothing is
        // lost -- and if they never arrive, the timeout fires at exactly the
        // rate samples are produced and the task reads them anyway. The
        // driver keeps working on hardware whose INT2 stays silent, at the
        // cost of waking the CPU per sample instead of being pushed.
        //
        // In WOM and OFF there is no sample stream to poll, so fall back to
        // the long watchdog: the only thing worth waking for there is a
        // motion interrupt, and those do arrive.
        uint32_t waitMs = kWatchdogMs;
        if (appliedMode_ == PowerMode::ACTIVITY || appliedMode_ == PowerMode::IMU6) {
            const float odr = dev_->accelOdrHz();
            if (odr > 0.0f) {
                waitMs = static_cast<uint32_t>(1000.0f / odr);
            }
        }
        TickType_t waitTicks = pdMS_TO_TICKS(waitMs);
        if (waitTicks == 0) {
            waitTicks = 1;  // never a zero-tick wait: that is a busy loop
        }

        ulTaskNotifyTake(pdTRUE, waitTicks);
        if (stopRequested_.load(std::memory_order_acquire)) {
            break;
        }

        // Say ONCE which path is actually feeding this driver, so a silent
        // fallback never gets mistaken for working interrupts.
        // Judge by the ISR COUNTER, never by whether the wait was notified.
        // setPowerMode() wakes this task with xTaskNotifyGive() from the app,
        // and the previous version counted that as interrupt activity -- it
        // printed "running on interrupts" on a board whose ISR had never run
        // once. A diagnostic that lies is worse than none.
        if (!pathReported_ &&
            (appliedMode_ == PowerMode::ACTIVITY || appliedMode_ == PowerMode::IMU6)) {
            ++pathSamples_;
            if (g_isrCount.load(std::memory_order_relaxed) > 0) {
                pathReported_ = true;
                ESP_LOGI(TAG, "dang chay bang NGAT tren GPIO%d",
                         static_cast<int>(config_.intPin));
            } else if (pathSamples_ > 20) {
                pathReported_ = true;
                ESP_LOGW(TAG, "GPIO%d khong sinh ngat -> chuyen sang POLL o %.1f Hz. "
                              "Van chay dung, chi ton CPU hon.",
                         static_cast<int>(config_.intPin), dev_->accelOdrHz());
            }
        }

        serviceInterrupt();
    }

    xSemaphoreGive(stopSemaphore_);
    vTaskDelete(nullptr);
}

void ImuManager::serviceInterrupt()
{
    const uint32_t t = nowMs();

    // ONE read. STATUS1 clears on read, so a second read to ask a second
    // question would silently drop events -- decode everything from this.
    StatusFlags f{};
    if (!dev_->readStatus(f)) {
        post(ImuEventMsg::Type::BUS_ERROR, t, 0, ESP_ERR_TIMEOUT);
        return;
    }

    if (f.accelReady || f.gyroReady) {
        RawSample raw{};
        if (dev_->readSample(raw)) {
            publish(raw, t);
            sampleCount_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    // A single interrupt can legitimately raise several of these at once.
    if (f.wakeOnMotion) post(ImuEventMsg::Type::WAKE_ON_MOTION, t);
    if (f.tap)          post(ImuEventMsg::Type::TAP, t);
    if (f.anyMotion)    post(ImuEventMsg::Type::ANY_MOTION, t);
    if (f.noMotion)     post(ImuEventMsg::Type::NO_MOTION, t);
    if (f.sigMotion)    post(ImuEventMsg::Type::SIG_MOTION, t);

    if (f.step) {
        uint32_t steps = 0;
        if (dev_->readStepCount(steps)) {
            stepCount_.store(steps, std::memory_order_relaxed);
            post(ImuEventMsg::Type::STEP, t, steps);
        }
    }
}

void ImuManager::publish(const RawSample& raw, uint32_t t)
{
    const float aScale = 1.0f / accelLsbPerG(dev_->accelRange());
    const float gScale = 1.0f / gyroLsbPerDps(dev_->gyroRange());
    const bool  gyroOn = dev_->gyroOdrHz() > 0.0f;

    float accelSensor[3];
    float gyroSensor[3];
    for (int i = 0; i < 3; ++i) {
        accelSensor[i] = static_cast<float>(raw.accel[i]) * aScale;
        gyroSensor[i]  = gyroOn ? static_cast<float>(raw.gyro[i]) * gScale : 0.0f;
    }

    sensors::Sample s{};
    config_.remap.apply(accelSensor, s.accelG);
    config_.remap.apply(gyroSensor, s.gyroDps);
    s.timestampMs = t;

    // Seqlock write: odd while in progress, even when consistent.
    const uint32_t seq = snapshotSeq_.load(std::memory_order_relaxed);
    snapshotSeq_.store(seq + 1, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_release);
    snapshot_ = s;
    std::atomic_thread_fence(std::memory_order_release);
    snapshotSeq_.store(seq + 2, std::memory_order_release);

    // The manager never learns what is on the other side of this call. That is
    // the whole reason components/imu does not depend on components/motion.
    if (config_.sink != nullptr) {
        config_.sink->onSample(s, t);
    }
}

void ImuManager::post(ImuEventMsg::Type type, uint32_t t, uint32_t steps,
                      esp_err_t err)
{
    if (eventQueue_ == nullptr) {
        return;
    }
    const ImuEventMsg msg{type, t, steps, err};
    // Timeout 0: dropping an event beats stalling the sensor task.
    if (xQueueSend(eventQueue_, &msg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "queue day, bo su kien %d", static_cast<int>(type));
    }
}

// -------------------------------------------------------------------- self test

void ImuManager::probeWhichIntPin()
{
    uint8_t ctrl8 = 0;
    if (!dev_->readRegister(reg::CTRL8, ctrl8)) {
        return;
    }

    // Handshake -> INT1 for the duration of the experiment.
    dev_->writeRegister(reg::CTRL8,
                        static_cast<uint8_t>(ctrl8 & ~reg::ctrl8::HANDSHAKE_VIA_STATUS));

    const int startLvl = gpio_get_level(config_.intPin);
    uint32_t  edges    = 0;

    // Fire the command WITHOUT ctrl9Command(): that polls STATUSINT, and here
    // the pin is the thing being watched.
    dev_->writeRegister(reg::CTRL9, reg::cmd::RST_FIFO);

    // Watch the pad WHILE waiting for the command to actually complete. The
    // first version sampled for a few microseconds and then read STATUSINT,
    // which still showed 0x00 -- the command had not finished, so INT1 had
    // never been raised and the "no reaction" result meant nothing. A probe
    // that cannot fail its own precondition is not a probe.
    uint8_t  si       = 0;
    bool     cmdDone  = false;
    for (int i = 0; i < 200 && !cmdDone; ++i) {
        for (int j = 0; j < 200; ++j) {
            if (gpio_get_level(config_.intPin) != startLvl) {
                ++edges;
                break;
            }
        }
        if (dev_->readRegister(reg::STATUSINT, si) &&
            (si & reg::statusint::CMD_DONE)) {
            cmdDone = true;
        }
    }
    dev_->writeRegister(reg::CTRL9, reg::cmd::ACK);
    dev_->writeRegister(reg::CTRL8, ctrl8);

    if (!cmdDone) {
        ESP_LOGE(TAG, "        -> phep thu INT1 KHONG ket luan duoc: lenh CTRL9 khong "
                      "hoan tat (STATUSINT=0x%02X), nen INT1 chua bao gio duoc keo len.",
                 si);
    } else if (edges > 0) {
        ESP_LOGE(TAG, "        -> GPIO%d PHAN UNG voi bat tay INT1! Chan nay noi vao "
                      "INT1, KHONG phai INT2.",
                 static_cast<int>(config_.intPin));
    } else {
        ESP_LOGE(TAG, "        -> lenh CTRL9 da hoan tat ma GPIO%d van dung yen o CA HAI "
                      "chan. Chip khang dinh ngat ben trong nhung khong ra toi pad.",
                 static_cast<int>(config_.intPin));
    }
}

void ImuManager::performRegisterSelfTest()
{
    // Runs ON THE IMU TASK. Everything here talks to the chip, and this device
    // has exactly one owner -- see i2c-bus-design.md section 7.1. An earlier
    // version did this from the caller's task and i2c::Device said so: two
    // CTRL9 handshakes interleaved and both failed.
    SelfTestResult r{};

    // 1+2. Identity and auto-increment together. WHO_AM_I and REVISION_ID are
    // adjacent, so one two-byte burst separates three faults: no chip, wrong
    // chip, and a register pointer that never advances.
    Qmi8658::IdentityReport ident{};
    dev_->identify(ident);
    r.identityOk      = ident.isQmi8658;
    r.autoIncrementOk = ident.autoIncrement;

    ESP_LOGI(TAG, "  [%s] nhan dang       WHO_AM_I=0x%02X (mong 0x05)",
             r.identityOk ? "PASS" : "FAIL", ident.whoAmI);
    ESP_LOGI(TAG, "  [%s] auto-increment  burst={0x%02X,0x%02X}, mong {0x%02X,0x%02X}%s",
             r.autoIncrementOk ? "PASS" : "FAIL", ident.burst[0], ident.burst[1],
             ident.whoAmI, ident.revision,
             ident.needsHighBitBurst ? "  (quy uoc reg|0x80)" : "");

    // 3. init() already refused to continue unless the reset marker appeared.
    uint8_t rst = 0;
    dev_->readRegister(reg::RESET_RESULT, rst);
    r.resetOk = true;
    ESP_LOGI(TAG, "  [PASS] reset          init() da xac nhan (0x4D doc lai=0x%02X)",
             rst);

    // 4. A CTRL9 round trip. The ONE check that proves CTRL8.bit7 is set: with
    // the default handshake the device signals completion on INT1, which this
    // board does not route, and the call would time out instead.
    // RST_FIFO, not ACK. CTRL_CMD_ACK (0x00) is what ENDS the protocol, not
    // a command the device executes -- it never raises CmdDone, so using it
    // as the probe made the one working mechanism look broken. Rev A has no
    // NOP; RST_FIFO is the cheapest harmless command, and the FIFO is unused.
    r.handshakeOk = dev_->ctrl9Command(reg::cmd::RST_FIFO);
    ESP_LOGI(TAG, "  [%s] bat tay CTRL9   qua STATUSINT.bit7%s",
             r.handshakeOk ? "PASS" : "FAIL",
             r.handshakeOk ? "" : "  <- kiem tra CTRL8.bit7");

    // 5. Ask the CHIP what mode it is in rather than trusting what we wrote.
    // This is what stands in for a current measurement: the registers say which
    // sensors run and how fast, and section 5 says what that costs in microamps.
    uint8_t ctrl2 = 0;
    uint8_t ctrl7 = 0;
    dev_->readRegister(reg::CTRL2, ctrl2);
    dev_->readRegister(reg::CTRL7, ctrl7);

    const bool accelOnly = (ctrl7 == reg::ctrl7::AEN);
    const bool odrCode   = ((ctrl2 & 0x0F) == 0x07);
    r.modeReadbackOk     = accelOnly && odrCode;
    ESP_LOGI(TAG, "  [%s] doc nguoc mode  CTRL2=0x%02X CTRL7=0x%02X (mong aODR=0x7, "
                  "chi aEN)",
             r.modeReadbackOk ? "PASS" : "FAIL", ctrl2, ctrl7);

    // 5b. Separate "the chip is not producing data" from "the chip produces
    //     data but the pin never moves". Polling STATUS0 over I2C bypasses
    //     the interrupt path entirely, so the two failures stop looking
    //     identical. Reading STATUS0 clears aDA, so each set bit means one
    //     fresh sample arrived since the previous poll.
    uint8_t ctrl1 = 0;
    uint8_t ctrl8 = 0;
    dev_->readRegister(reg::CTRL1, ctrl1);
    dev_->readRegister(reg::CTRL8, ctrl8);

    uint8_t fifoCtrl = 0;
    dev_->readRegister(reg::FIFO_CTRL, fifoCtrl);

    const uint32_t isrBefore = g_isrCount.load(std::memory_order_relaxed);
    uint32_t adaCount   = 0;
    uint32_t int2Mirror = 0;
    uint32_t pinEdges = 0;
    int      lastLvl  = gpio_get_level(config_.intPin);
    for (int i = 0; i < 100; ++i) {
        uint8_t s0 = 0;
        if (dev_->readRegister(reg::STATUS0, s0) && (s0 & reg::status0::ADA)) {
            ++adaCount;
        }
        const int lvl = gpio_get_level(config_.intPin);
        if (lvl != lastLvl) {
            ++pinEdges;
            lastLvl = lvl;
        }

        // Rev A section 6.2: with SyncSample off, STATUSINT.bit0 mirrors the
        // INT2 pin and bit1 mirrors INT1. This asks the CHIP what it thinks
        // its own output is doing, which separates "the chip never asserts",
        // "it asserts but the pad does not drive" and "the pad drives but
        // the ESP32 misses it" -- three faults that look identical from here.
        uint8_t si = 0;
        if (dev_->readRegister(reg::STATUSINT, si) &&
            (si & reg::statusint::AVAIL)) {
            ++int2Mirror;
        }
        delayMs(2);
    }

    const uint32_t isrHits = g_isrCount.load(std::memory_order_relaxed) - isrBefore;

    ESP_LOGI(TAG, "  [--]  duong du lieu   CTRL1=0x%02X CTRL8=0x%02X FIFO_CTRL=0x%02X",
             ctrl1, ctrl8, fifoCtrl);
    ESP_LOGI(TAG, "  [--]  100 lan doc:   aDA len %" PRIu32 " | STATUSINT.INT2 len "
                  "%" PRIu32 " | GPIO%d doi muc %" PRIu32 " | ISR chay %" PRIu32,
             adaCount, int2Mirror, static_cast<int>(config_.intPin), pinEdges,
             isrHits);

    if (adaCount == 0) {
        ESP_LOGE(TAG, "        -> cam bien KHONG sinh du lieu. Van de o che do/ODR, "
                      "khong phai o chan ngat.");
    } else if (isrHits > 0) {
        ESP_LOGW(TAG, "        -> chan CO xung (ISR chay %" PRIu32 " lan) nhung qua hep "
                      "de gpio_get_level() bat duoc. Duong ngat OK.", isrHits);
    } else if (int2Mirror == 0) {
        ESP_LOGE(TAG, "        -> chip CO du lieu nhung CHINH NO bao INT2 luon o muc "
                      "thap (STATUSINT.bit0). Khong phai loi mach hay loi ESP32: chip "
                      "khong he khang dinh DRDY ra INT2.");
    } else if (pinEdges == 0) {
        // The chip drives its INT2 internally but the pad stays put. The one
        // remaining question is which pin GPIO38 is actually bonded to --
        // Waveshare's own documents disagree, some calling it INT1.
        //
        // Rev A section 5.10: with CTRL8.bit7 == 0 the CTRL9 handshake raises
        // INT1. So point the handshake at INT1, run a command, and watch the
        // pad. If it moves, GPIO38 is INT1 and every interrupt this driver
        // configures has been going to a pin nobody is listening to.
        probeWhichIntPin();
    } else if (false) {
        ESP_LOGE(TAG, "        -> chip bao INT2 CO hoat dong (%" PRIu32 " lan) nhung "
                      "chan GPIO%d khong nhuc nhich -> nghi ngo pad/mach ngoai.",
                 int2Mirror, static_cast<int>(config_.intPin));
    } else {
        ESP_LOGW(TAG, "        -> chan CO dao muc; neu ODR van dem 0 thi van de nam o "
                      "phia ESP32 (gpio_config / ISR), khong phai o chip.");
    }

    selfTestPartial_ = r;
}

esp_err_t ImuManager::runSelfTest(SelfTestResult& out)
{
    if (!started_) {
        return ESP_ERR_INVALID_STATE;
    }
    out = SelfTestResult{};

    ESP_LOGI(TAG, "--- self test (khong can dung cu do nao) ---");

    // Get into the mode being measured BEFORE anything is checked, and let the
    // task actually reach it. The previous version read the registers back while
    // the task was still applying a different mode, then blamed the chip.
    setPowerMode(PowerMode::ACTIVITY);
    for (int i = 0; i < 50 && appliedMode_ != PowerMode::ACTIVITY; ++i) {
        delayMs(20);
    }

    // Phase A: hand the register work to the task that owns the device.
    selfTestRequested_.store(true, std::memory_order_release);
    xTaskNotifyGive(taskHandle_);
    if (xSemaphoreTake(selfTestDone_, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGE(TAG, "task IMU khong tra loi self test");
        return ESP_ERR_TIMEOUT;
    }
    out = selfTestPartial_;

    // Phase B: the caller's own work. Counting interrupts and reading latest()
    // touch atomics only, never the bus -- and they REQUIRE the task to be
    // servicing interrupts normally, which it cannot do while running phase A.

    // 6. Count real interrupts over a known window. This is the stand-in for a
    // frequency meter, and because ODR sets the current draw it is an indirect
    // check on the power budget too.
    const uint32_t before    = sampleCount_.load(std::memory_order_relaxed);
    const uint32_t isrBefore = g_isrCount.load(std::memory_order_relaxed);
    const int64_t  t0        = esp_timer_get_time();
    delayMs(1000);
    const int64_t  t1    = esp_timer_get_time();
    const uint32_t after = sampleCount_.load(std::memory_order_relaxed);

    const uint32_t isrAfter = g_isrCount.load(std::memory_order_relaxed);
    const float seconds = static_cast<float>(t1 - t0) / 1000000.0f;
    out.measuredOdrHz   = static_cast<float>(after - before) / seconds;
    const float isrHz   = static_cast<float>(isrAfter - isrBefore) / seconds;
    const float expected = accelOdrHz();
    out.odrOk = expected > 0.0f &&
                std::fabs(out.measuredOdrHz - expected) < expected * 0.15f;
    ESP_LOGI(TAG, "  [%s] ODR that        dem duoc %.1f Hz (ISR %.1f Hz), khai bao "
                  "%.1f Hz",
             out.odrOk ? "PASS" : "FAIL", out.measuredOdrHz, isrHz, expected);
    if (out.measuredOdrHz == 0.0f) {
        ESP_LOGE(TAG, "        -> %s",
                 (isrHz > 1.0f)
                     ? "ISR CO chay: chan hoat dong, loi nam o buoc doc mau"
                     : "ISR khong chay lan nao: chan that su dung yen");
    }

    // 7. Gravity. A watch lying still reads 1 g whichever way up it is, so this
    // single assertion covers the full-scale setting, the byte order and the
    // sign handling at once, in any orientation.
    sensors::Sample s{};
    if (latest(s)) {
        out.measuredG = std::sqrt(s.accelG[0] * s.accelG[0] +
                                  s.accelG[1] * s.accelG[1] +
                                  s.accelG[2] * s.accelG[2]);
        // +/-0.15 g, not +/-0.05. Rev A specifies an Initial Offset Tolerance
        // of +/-100 mg for an uncalibrated part, plus +/-6% initial sensitivity
        // tolerance. A tighter window than the datasheet allows would fail a
        // perfectly good chip -- the same mistake the hard-coded REVISION_ID
        // check made. This test exists to catch a wrong full-scale setting or
        // a byte-order error, and both of those are off by 2x or more.
        out.gravityOk = out.measuredG > 0.85f && out.measuredG < 1.15f;
    }
    ESP_LOGI(TAG, "  [%s] do lon trong luc |a| = %.3f g (mong 1.00 +/- 0.15; "
                  "offset xuat xuong cua chip da la +/-0.10)",
             out.gravityOk ? "PASS" : "FAIL", out.measuredG);

    ESP_LOGI(TAG, "--- self test: %s ---", out.allPassed() ? "TAT CA DAT" : "CO LOI");
    return out.allPassed() ? ESP_OK : ESP_FAIL;
}

}  // namespace imu
