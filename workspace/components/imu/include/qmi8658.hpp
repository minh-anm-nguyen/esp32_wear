// Pure logic layer for the QMI8658C.
//
// Includes nothing from ESP-IDF. Its only window on the world is an i2c::Bus&,
// which is what lets the CTRL9 handshake, the WoM setup sequence and the ODR
// arithmetic all be tested on a PC against i2c::FakeBus -- in the right ORDER,
// with no chip.
//
// It also has NO way to sleep. That is deliberate: applyMode() returns the
// settling time the caller must honour instead of blocking, so this layer stays
// free of FreeRTOS and the settling time itself becomes an assertable value.
//
// See doc-design/imu-qmi8658c-design.md sections 10 and 13.
#pragma once

#include "qmi8658_regs.hpp"

#include "i2c_bus.hpp"

namespace imu {

// ------------------------------------------------------------------ data types

// The application says which state it is in; this class turns that into
// CTRL2/CTRL3/CTRL7/CTRL8. Exposing raw ODR instead would eventually leave a
// code path with the gyro still on, and a watch battery does not forgive that.
enum class PowerMode : uint8_t {
    OFF,       // CTRL7 = 0. NOT sensorDisable: coming back from that costs 1.75 s
    WOM,       // accel low-power 11 Hz + wake-on-motion  ~35 uA
    ACTIVITY,  // accel only, 62.5 Hz                     ~133 uA
    IMU6,      // accel + gyro, 112.1 Hz                  ~750 uA
};

enum class AccelRange : uint8_t { G2 = 0, G4 = 1, G8 = 2, G16 = 3 };

// Rev A stops at +/-1024 dps; code 7 is N/A. Rev 0.6's +/-2048 does not exist.
enum class GyroRange : uint8_t {
    DPS16 = 0, DPS32 = 1, DPS64 = 2, DPS128 = 3,
    DPS256 = 4, DPS512 = 5, DPS1024 = 6,
};

enum class Ctrl9Handshake : uint8_t { Int1Pin, StatusRegister };
enum class IntPin : uint8_t { Int1, Int2 };

// Raw, straight off the wire: no unit conversion, no axis remap.
struct RawSample {
    int16_t accel[3]{};
    int16_t gyro[3]{};
};

// What one STATUS read said. Produced by a SINGLE readStatus() call, because
// reading STATUS1 clears its flags -- ask once, decode everything.
struct StatusFlags {
    bool accelReady{false};
    bool gyroReady{false};
    bool wakeOnMotion{false};
    bool step{false};
    bool tap{false};
    bool anyMotion{false};
    bool noMotion{false};
    bool sigMotion{false};
};

struct WomConfig {
    uint16_t thresholdMg{200};   // 1 mg per LSB, 0 disables WoM entirely
    uint8_t  blankingSamples{8}; // accel samples ignored after enabling, 0..63
    IntPin   intPin{IntPin::Int2};
};

// -------------------------------------------------------------- unit conversion

// Rev A Table 7/8. Both halve per step, so a shift is exact -- no lookup table
// that could drift out of sync with the enum.
constexpr float accelLsbPerG(AccelRange r)
{
    return static_cast<float>(16384u >> static_cast<uint8_t>(r));
}

constexpr float gyroLsbPerDps(GyroRange r)
{
    return static_cast<float>(2048u >> static_cast<uint8_t>(r));
}

constexpr float tempLsbToC(int16_t raw) { return static_cast<float>(raw) / 256.0f; }

// THE trap of this chip. Rev A Table 22 gives aODR two columns: the same
// register value means one frequency with the gyro off and another with it on,
// because in 6DOF the accelerometer is retimed off the gyro's oscillator.
// Enable the gyro and the accelerometer silently drops 125 -> 112.1 Hz, an 11%
// error in any dt computed from a constant. Section 4.2.
float accelOdrHzFor(uint8_t aOdrCode, bool gyroEnabled);
float gyroOdrHzFor(uint8_t gOdrCode);

// ---------------------------------------------------------------------- Qmi8658

class Qmi8658 {
public:
    // Optional 1 ms sleep, used only while polling for a CTRL9 completion. Left
    // null the polling is a tight loop, which is exactly what a host test wants.
    using DelayFn = void (*)(uint32_t ms);

    explicit Qmi8658(i2c::Bus& bus, DelayFn delay = nullptr);

    // ---------------------------------------------------------- identification

    // What identify() found, so a failure can be explained rather than just
    // reported. Every field is filled in even when the outcome is a failure.
    struct IdentityReport {
        bool    read{false};        // did the bus answer at all
        uint8_t whoAmI{0};          // single-byte read of 0x00
        uint8_t revision{0};        // single-byte read of 0x01
        uint8_t burst[2]{0, 0};     // two-byte burst from 0x00
        bool    isQmi8658{false};   // whoAmI == 0x05
        bool    autoIncrement{false};   // burst[1] advanced past 0x00
        bool    needsHighBitBurst{false};  // ...only once 0x80 was ORed in
    };

    // Staged identification, because "probe failed" on its own names three
    // very different faults and fixes none of them.
    //
    //   1. read 0x00 alone      -> is this even a QMI8658C
    //   2. read 0x01 alone      -> what revision, whatever it happens to be
    //   3. burst 0x00, 2 bytes  -> did the register pointer advance
    //   4. burst 0x80, 2 bytes  -> if not, does the OTHER auto-increment
    //                              convention work (Rev 0.6 section 12.2 says
    //                              bit 7 of the register address enables it,
    //                              Rev A says CTRL1.bit6 -- the two revisions
    //                              disagree and only silicon settles it)
    //
    // Step 3 compares against the byte read in step 2, NOT against a hard
    // coded 0x68. A part whose revision differs is not a broken part, and an
    // over-specified check would have condemned it.
    //
    // If step 4 succeeds, every later multi-byte access switches to that
    // convention automatically.
    bool identify(IdentityReport& report);

    // WHO_AM_I == 0x05, nothing more. Convenience over identify().
    bool probe();

    bool readRegister(uint8_t reg, uint8_t& value);
    bool writeRegister(uint8_t reg, uint8_t value);

    // Write 0xB0 to RESET, then confirm register 0x4D reads 0x80. The caller
    // must allow up to 15 ms between the two -- hence the split.
    bool beginReset();
    bool resetSucceeded();

    // ------------------------------------------------------------- board setup

    // Three neutral primitives rather than one prepareForNoInt1(): which value
    // is correct is a fact about the BOARD, and a chip driver has no business
    // knowing that this particular PCB left INT1 unrouted. Section 10.
    bool setAddressAutoIncrement(bool on);
    bool setCtrl9Handshake(Ctrl9Handshake mode);
    bool setActivityIntPin(IntPin pin);

    // ------------------------------------------------------------------- modes

    void setRanges(AccelRange accel, GyroRange gyro);
    void setWomConfig(const WomConfig& wom) { wom_ = wom; }

    // Applies the mode and reports how long the caller must wait before the
    // data is trustworthy: sensor wake-up plus three filter periods. Returns
    // false without changing mode_ if any register write failed.
    bool applyMode(PowerMode mode, uint32_t& settleMsOut);

    PowerMode mode() const { return mode_; }

    // The frequency actually in force, which is NOT a constant -- see
    // accelOdrHzFor(). Anything integrating samples must ask this, every time.
    float accelOdrHz() const;
    float gyroOdrHz() const;

    // ----------------------------------------------------------------- reading

    // One 12-byte burst, AX_L..GZ_H.
    bool readSample(RawSample& out);
    bool readTemperature(int16_t& raw);

    // STATUS0 and STATUS1 in one two-byte burst, decoded together. Call once
    // per interrupt: STATUS1 is read-to-clear.
    bool readStatus(StatusFlags& out);

    bool readStepCount(uint32_t& steps);

    // ------------------------------------------------------------- CTRL9 things

    // Runs the whole documented WoM procedure (section 14.2): disable sensors,
    // set the accelerometer to low power, load CAL1, issue the CTRL9 command,
    // re-enable the accelerometer. applyMode(WOM) calls this.
    bool configureWakeOnMotion();

    // Rev A section 12.6, and it is NOT optional.
    //
    // Writing a new CTRL2/CTRL7 does not take the part out of WoM: the mode
    // ends only when the threshold is set to zero and the CTRL9 command is
    // re-issued. Leave it out and every register reads back exactly as the
    // new mode intends while the chip quietly keeps behaving as WoM -- no
    // data-ready, no samples, just motion interrupts. Nothing anywhere else
    // reveals it.
    bool disableWakeOnMotion();

    bool womArmed() const { return womArmed_; }

    // The handshake behind every complex operation: write params, write the
    // command, wait for STATUSINT.bit7, then ACK. Skipping the ACK leaves the
    // next command dead in the water.
    bool ctrl9Command(uint8_t command);

    // Accessors used by tests and by the manager's self-test.
    AccelRange accelRange() const { return accelRange_; }
    GyroRange  gyroRange() const { return gyroRange_; }

private:
    bool readReg(uint8_t reg, uint8_t& value);

    // Multi-byte access, honouring whichever auto-increment convention
    // identify() found this part to use.
    bool burstRead(uint8_t reg, uint8_t* dst, std::size_t len);
    bool burstWrite(uint8_t reg, const uint8_t* src, std::size_t len);
    bool updateReg(uint8_t reg, uint8_t clearMask, uint8_t setMask);

    // Encoded aODR/gODR nibbles for the current mode.
    uint8_t accelOdrCode() const { return aOdrCode_; }

    i2c::Bus& bus_;
    DelayFn   delay_;

    PowerMode  mode_{PowerMode::OFF};
    AccelRange accelRange_{AccelRange::G4};   // wrist flicks pass 2 g, rarely 4
    GyroRange  gyroRange_{GyroRange::DPS256}; // a fast wrist turn is 200-400 dps
    WomConfig  wom_{};

    uint8_t aOdrCode_{0x07};  // 62.5 Hz accel-only
    uint8_t gOdrCode_{0x06};  // 112.1 Hz
    bool    gyroEnabled_{false};

    // Set by identify() when the part only auto-increments with bit 7 of
    // the register address set. Defaults off: Rev A and SensorLib both say
    // CTRL1.ADDR_AI is enough.
    bool    burstHighBit_{false};

    // Tracks whether the part is actually in WoM mode, which is not the
    // same question as "what mode did we last ask for".
    bool    womArmed_{false};
};

}  // namespace imu
