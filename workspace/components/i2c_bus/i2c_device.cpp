#include "i2c_device.hpp"

#include <cinttypes>

#include "esp_log.h"
#include "soc/soc_caps.h"

namespace i2c {

namespace {

constexpr const char* TAG = "i2c";

// Known inhabitants of this board's bus, so logScan() can say what it found
// instead of printing bare numbers at someone doing bring-up.
struct KnownDevice {
    uint16_t    address;
    const char* name;
};

constexpr KnownDevice kKnown[] = {
    {0x15, "CST816T (cam ung)"},
    {0x51, "PCF85063 (RTC)"},
    {0x6B, "QMI8658C (IMU)"},
};

const char* nameOf(uint16_t address)
{
    for (const auto& k : kKnown) {
        if (k.address == address) {
            return k.name;
        }
    }
    return "(khong ro)";
}

}  // namespace

// ----------------------------------------------------------------------- Device

void Device::checkOwningTask()
{
    TaskHandle_t self = xTaskGetCurrentTaskHandle();

    if (owner_ == nullptr) {
        owner_ = self;  // first caller claims it
        return;
    }
    if (owner_ == self || ownerWarned_) {
        return;
    }

    // Once only: a violated invariant repeated at 100 Hz would bury the log it
    // is trying to draw attention to.
    ownerWarned_ = true;
    ESP_LOGE(TAG,
             "0x%02X bi TRUY CAP TU HAI TASK ('%s' va '%s'). Khoa bus cua IDF chi "
             "bao ve MOT giao dich, nen chuoi doc-sua-ghi hay bat tay CTRL9 se dan "
             "xen. Xem i2c-bus-design.md muc 7.1.",
             address_, pcTaskGetName(owner_), pcTaskGetName(self));
}

bool Device::transfer(const uint8_t* tx, std::size_t txLen,
                      uint8_t* rx, std::size_t rxLen)
{
    if (dev_ == nullptr) {
        lastError_ = ESP_ERR_INVALID_STATE;
        return false;
    }

    checkOwningTask();

    if (!sizeWarned_ && (txLen + rxLen) > maxBytesFor5ms_) {
        sizeWarned_ = true;
        ESP_LOGW(TAG,
                 "0x%02X: giao dich %u byte vuot ngan sach ~5 ms (%u byte). Cam ung "
                 "se bi chan trong suot thoi gian do. Xem muc 8.",
                 address_, static_cast<unsigned>(txLen + rxLen),
                 static_cast<unsigned>(maxBytesFor5ms_));
    }

    // Never -1: that is portMAX_DELAY, and one dead device would then hang every
    // task on the bus for good. Section 7.3.
    const esp_err_t err =
        (rxLen > 0)
            ? i2c_master_transmit_receive(dev_, tx, txLen, rx, rxLen, kXferTimeoutMs)
            : i2c_master_transmit(dev_, tx, txLen, kXferTimeoutMs);

    lastError_ = err;
    if (err == ESP_OK) {
        consecutiveErrors_ = 0;
        return true;
    }

    if (consecutiveErrors_ < UINT32_MAX) {
        ++consecutiveErrors_;
    }
    // Recorded before anything decides whether this failure is worth a line.
    // The isolated one never is -- and it was exactly the isolated one that
    // left no evidence to attribute later.
    if (totalErrors_ < UINT32_MAX) {
        ++totalErrors_;
    }
    if (consecutiveErrors_ > worstErrorStreak_) {
        worstErrorStreak_ = consecutiveErrors_;
    }
    lastFailure_   = err;
    lastFailureMs_ = static_cast<uint32_t>(pdTICKS_TO_MS(xTaskGetTickCount()));

    // Exactly at the threshold, once: enough to explain a silent device without
    // flooding the log while it stays dead.
    if (consecutiveErrors_ == kDeadThreshold) {
        ESP_LOGE(TAG, "0x%02X: %" PRIu32 " loi lien tiep (%s) -> coi nhu hong",
                 address_, consecutiveErrors_, esp_err_to_name(err));
    }
    return false;
}

bool Device::readRegs(uint8_t reg, uint8_t* dst, std::size_t len)
{
    if (dst == nullptr || len == 0) {
        lastError_ = ESP_ERR_INVALID_ARG;
        return false;
    }
    // One transaction with a repeated START, never a write then a separate read.
    return transfer(&reg, 1, dst, len);
}

bool Device::writeReg(uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = {reg, value};
    return transfer(buf, sizeof(buf), nullptr, 0);
}

bool Device::writeRegs(uint8_t reg, const uint8_t* src, std::size_t len)
{
    if (src == nullptr || len == 0) {
        lastError_ = ESP_ERR_INVALID_ARG;
        return false;
    }

    // The register pointer and the payload have to leave as ONE transaction, so
    // they need to be contiguous. 32 bytes covers every multi-register write on
    // this board with room to spare (the largest is 8: QMI8658C CAL1_L..CAL4_H).
    constexpr std::size_t kMaxPayload = 31;
    if (len > kMaxPayload) {
        lastError_ = ESP_ERR_INVALID_SIZE;
        ESP_LOGE(TAG, "0x%02X: writeRegs %u byte > toi da %u", address_,
                 static_cast<unsigned>(len), static_cast<unsigned>(kMaxPayload));
        return false;
    }

    uint8_t buf[1 + kMaxPayload];
    buf[0] = reg;
    for (std::size_t i = 0; i < len; ++i) {
        buf[1 + i] = src[i];
    }
    return transfer(buf, len + 1, nullptr, 0);
}

// ------------------------------------------------------------------ BusManager

BusManager::~BusManager()
{
    deinit();
}

esp_err_t BusManager::init(const Config& config)
{
    if (bus_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_master_bus_config_t cfg{};
    cfg.i2c_port                     = config.port;
    cfg.sda_io_num                   = config.sda;
    cfg.scl_io_num                   = config.scl;
    cfg.clk_source                   = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt            = config.glitchIgnoreCnt;
    cfg.flags.enable_internal_pullup = config.enableInternalPullup ? 1u : 0u;

    // Never hand the driver a flag this chip cannot honour: it does not
    // degrade gracefully, it refuses to create the bus at all. Clamp here so a
    // config written for a different target cannot turn into a boot loop.
#if SOC_I2C_SUPPORT_SLEEP_RETENTION
    cfg.flags.allow_pd = config.allowPowerDown ? 1u : 0u;
#else
    if (config.allowPowerDown) {
        ESP_LOGW(TAG, "chip nay khong ho tro I2C sleep retention -- bo qua "
                      "allowPowerDown thay vi de bus khong tao duoc");
    }
    cfg.flags.allow_pd = 0;
#endif

    const esp_err_t err = i2c_new_master_bus(&cfg, &bus_);
    if (err != ESP_OK) {
        bus_ = nullptr;
        ESP_LOGE(TAG, "i2c_new_master_bus(SDA=%d, SCL=%d) that bai: %s",
                 static_cast<int>(config.sda), static_cast<int>(config.scl),
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "bus san sang: SDA=GPIO%d, SCL=GPIO%d",
             static_cast<int>(config.sda), static_cast<int>(config.scl));
    return ESP_OK;
}

void BusManager::deinit()
{
    if (bus_ == nullptr) {
        return;
    }
    for (std::size_t i = 0; i < deviceCount_; ++i) {
        if (devices_[i]->dev_ != nullptr) {
            i2c_master_bus_rm_device(devices_[i]->dev_);
            devices_[i]->dev_ = nullptr;
        }
    }
    deviceCount_ = 0;
    i2c_del_master_bus(bus_);
    bus_ = nullptr;
}

esp_err_t BusManager::createDevice(uint16_t address, uint32_t sclHz, Device& out)
{
    if (bus_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (deviceCount_ >= kMaxDevices) {
        return ESP_ERR_NO_MEM;
    }
    if (out.dev_ != nullptr) {
        return ESP_ERR_INVALID_STATE;  // already bound
    }

    i2c_device_config_t cfg{};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address  = address;
    cfg.scl_speed_hz    = sclHz;

    const esp_err_t err = i2c_master_bus_add_device(bus_, &cfg, &out.dev_);
    if (err != ESP_OK) {
        out.dev_ = nullptr;
        ESP_LOGE(TAG, "them thiet bi 0x%02X that bai: %s", address,
                 esp_err_to_name(err));
        return err;
    }

    out.address_ = address;
    // 5 ms of wire time at this device's own clock, ~9 bits per byte.
    out.maxBytesFor5ms_ = (sclHz / 1000u) * 5u / 9u;

    devices_[deviceCount_++] = &out;
    return ESP_OK;
}

esp_err_t BusManager::probe(uint16_t address)
{
    if (bus_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_probe(bus_, address, kXferTimeoutMs);
}

std::size_t BusManager::scan(uint16_t* found, std::size_t maxFound)
{
    if (bus_ == nullptr || found == nullptr) {
        return 0;
    }
    std::size_t n = 0;
    for (uint16_t addr = 0x08; addr <= 0x77 && n < maxFound; ++addr) {
        if (i2c_master_probe(bus_, addr, 20) == ESP_OK) {
            found[n++] = addr;
        }
    }
    return n;
}

void BusManager::logScan(const char* tag) const
{
    if (bus_ == nullptr) {
        ESP_LOGE(tag, "bus chua init");
        return;
    }

    uint16_t          found[16];
    std::size_t       n = 0;
    // const_cast: scan() only reads the bus, but the IDF probe API is non-const.
    n = const_cast<BusManager*>(this)->scan(found, sizeof(found) / sizeof(found[0]));

    ESP_LOGI(tag, "--- quet bus I2C: tim thay %u thiet bi ---",
             static_cast<unsigned>(n));
    for (std::size_t i = 0; i < n; ++i) {
        ESP_LOGI(tag, "  0x%02X  %s", found[i], nameOf(found[i]));
    }

    // Say what is MISSING too. A scan that prints two lines looks like a success
    // until you remember there should have been three.
    for (const auto& k : kKnown) {
        bool present = false;
        for (std::size_t i = 0; i < n; ++i) {
            if (found[i] == k.address) {
                present = true;
                break;
            }
        }
        if (!present) {
            ESP_LOGE(tag, "  THIEU 0x%02X  %s", k.address, k.name);
        }
    }
}

esp_err_t BusManager::recover()
{
    if (bus_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "phuc hoi bus...");

    // Let whatever is in flight finish or time out before yanking the bus.
    esp_err_t err = i2c_master_bus_wait_all_done(bus_, kXferTimeoutMs);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "wait_all_done: %s", esp_err_to_name(err));
    }

    err = i2c_master_bus_reset(bus_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_reset() that bai: %s", esp_err_to_name(err));
        return err;
    }

    // Clear the health counters only for devices that answer again, so a chip
    // that is genuinely dead stays marked as such.
    for (std::size_t i = 0; i < deviceCount_; ++i) {
        Device* d = devices_[i];
        if (i2c_master_probe(bus_, d->address_, kXferTimeoutMs) == ESP_OK) {
            d->consecutiveErrors_ = 0;
            d->lastError_         = ESP_OK;
        } else {
            ESP_LOGE(TAG, "  0x%02X van khong tra loi sau khi reset", d->address_);
        }
    }
    return ESP_OK;
}

}  // namespace i2c
