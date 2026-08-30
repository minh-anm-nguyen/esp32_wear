// ESP-IDF side of the shared I2C bus: one owner for the bus handle, one Device
// per chip, and the policies from doc-design/i2c-bus-design.md enforced in code
// rather than only in prose.
//
// Three of those rules are checked at runtime here, because a rule that only
// lives in a document is a rule that gets broken silently:
//   section 7.1  one task owns one device   -> Device logs a violation
//   section 7.3  never an infinite timeout  -> not even expressible in this API
//   section 7.4  no transaction over 5 ms   -> Device logs an oversize transfer
#pragma once

#include "i2c_bus.hpp"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace i2c {

// Applied to every transfer. Deliberately not a parameter: if each driver picks
// its own, a slow or dead device silently spends another driver's latency
// budget, because the IDF bus lock is taken with THIS call's timeout.
inline constexpr int kXferTimeoutMs = 50;

// 5 ms of bus time at 400 kHz, at ~9 bits per byte. Crossing it does not fail
// the transfer -- it logs, because the cost lands on the touch panel, not here.
inline constexpr std::size_t kSoftMaxTransferBytes = 200;

// Consecutive failures before a device is declared unhealthy. One NACK is
// noise; five in a row is a dead chip or a wedged bus, and the two are told
// apart by asking whether the OTHER devices are failing too.
inline constexpr uint32_t kDeadThreshold = 5;

// ----------------------------------------------------------------------- Device

class Device : public Bus {
public:
    Device() = default;

    // Owns a bus-level handle, so copying it would double-free.
    Device(const Device&)            = delete;
    Device& operator=(const Device&) = delete;

    bool readRegs(uint8_t reg, uint8_t* dst, std::size_t len) override;
    bool writeReg(uint8_t reg, uint8_t value) override;
    bool writeRegs(uint8_t reg, const uint8_t* src, std::size_t len) override;

    bool      valid() const { return dev_ != nullptr; }
    uint16_t  address() const { return address_; }
    esp_err_t lastError() const { return lastError_; }
    uint32_t  consecutiveErrors() const { return consecutiveErrors_; }
    bool      healthy() const { return consecutiveErrors_ < kDeadThreshold; }

    // Hand the device to the calling task.
    //
    // The one-task-one-device rule has a legitimate exception: a driver is
    // normally configured from app_main and only THEN handed to the task that
    // will own it from there on. Without a way to say so, that perfectly
    // correct sequence trips the very check meant to catch real violations --
    // and a warning that cries wolf is worse than no warning.
    //
    // Call this from the new owner, once, before it touches the device.
    void claimOwnership()
    {
        owner_       = xTaskGetCurrentTaskHandle();
        ownerWarned_ = false;
    }

private:
    friend class BusManager;

    // Returns false and sets lastError_ on failure, so the Bus contract can stay
    // a plain bool and the pure-logic layer never sees esp_err_t.
    bool transfer(const uint8_t* tx, std::size_t txLen, uint8_t* rx, std::size_t rxLen);

    // Section 7.1. The first task to use this device claims it; a second one
    // gets an error log. Not an assert: a wrong-task read is a design bug worth
    // shouting about, but killing a watch over it is worse than the bug.
    void checkOwningTask();

    i2c_master_dev_handle_t dev_{nullptr};
    uint16_t                address_{0};
    std::size_t             maxBytesFor5ms_{kSoftMaxTransferBytes};

    TaskHandle_t owner_{nullptr};
    bool         ownerWarned_{false};
    bool         sizeWarned_{false};

    esp_err_t lastError_{ESP_OK};
    uint32_t  consecutiveErrors_{0};
};

// ------------------------------------------------------------------ BusManager

class BusManager {
public:
    struct Config {
        gpio_num_t sda{GPIO_NUM_11};   // ESP32-S3-Touch-LCD-1.69 V2.1
        gpio_num_t scl{GPIO_NUM_10};
        int        port{-1};           // -1: let the driver choose
        uint8_t    glitchIgnoreCnt{7}; // IDF's typical value

        // The board already has external pull-ups. The internal ones are far
        // too weak for 400 kHz, so enabling them would only mask a wiring fault
        // until the day it matters.
        bool enableInternalPullup{false};

        // flags.allow_pd: back up and restore the I2C registers around sleep so
        // the peripheral's power domain can be switched off.
        //
        // FALSE on purpose. The ESP32-S3 does not define
        // SOC_I2C_SUPPORT_SLEEP_RETENTION, and i2c_new_master_bus() REJECTS a
        // non-zero allow_pd outright on such a target -- it does not warn and
        // carry on, it returns ESP_ERR_NOT_SUPPORTED. Requesting a flag the
        // silicon lacks is not a missed optimisation, it is a failed boot.
        //
        // init() clamps this to 0 on unsupported targets anyway, so setting it
        // true is harmless -- but there is nothing on this board to gain by it.
        bool allowPowerDown{false};
    };

    BusManager() = default;
    ~BusManager();

    BusManager(const BusManager&)            = delete;
    BusManager& operator=(const BusManager&) = delete;

    // Call once, from app_main. Section 7.5: nobody else creates a bus.
    esp_err_t init(const Config& config);
    void      deinit();

    bool isReady() const { return bus_ != nullptr; }

    // Binds a chip to this bus. The Device must outlive every driver holding a
    // reference to it -- give it static storage or keep it in app_main's frame.
    esp_err_t createDevice(uint16_t address, uint32_t sclHz, Device& out);

    esp_err_t probe(uint16_t address);

    // Sweeps 0x08..0x77 and fills `found`. Returns how many answered.
    std::size_t scan(uint16_t* found, std::size_t maxFound);

    // Bring-up aid: scan and print what answered, with the three expected
    // addresses called out by name. Proves the wiring without any instrument.
    void logScan(const char* tag) const;

    // Section 9.2. Waits for in-flight transfers, then resets the bus and
    // re-probes every registered device.
    esp_err_t recover();

    // Escape hatch for a third-party driver that insists on the raw handle.
    i2c_master_bus_handle_t handle() const { return bus_; }

private:
    static constexpr std::size_t kMaxDevices = 8;

    i2c_master_bus_handle_t bus_{nullptr};
    Device*                 devices_[kMaxDevices]{};
    std::size_t             deviceCount_{0};
};

}  // namespace i2c
