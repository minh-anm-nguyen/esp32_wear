#include "qmi8658.hpp"

namespace imu {

namespace {

// aODR / gODR codes used by the four power modes.
constexpr uint8_t kAOdr62_5   = 0x07;  // accel-only 62.5 Hz  (56.05 Hz in 6DOF)
constexpr uint8_t kAOdr6Dof   = 0x06;  // 6DOF: 112.1 Hz
constexpr uint8_t kAOdrLp11   = 0x0E;  // low power 11 Hz -- accel-only, gyro OFF
constexpr uint8_t kGOdr112_1  = 0x06;

// Rev A section 7.2. Filter settling is 3/ODR on top of the wake-up time.
constexpr uint32_t kAccelWakeMs = 3;
constexpr uint32_t kGyroWakeMs  = 60;

// CTRL9 completion poll. 100 tries at 1 ms is far beyond the microseconds a
// command actually takes, and still bounded -- an unACKed command must never
// hang the IMU task.
constexpr uint32_t kCtrl9MaxPolls = 100;

// Rev A Table 22, column "ODR Rate (Accel only)".
constexpr float kAccelOnlyOdr[9] = {
    0.0f,     // 0000 N/A when the gyro is off
    0.0f,     // 0001 N/A
    0.0f,     // 0010 N/A
    1000.0f,  // 0011
    500.0f,   // 0100
    250.0f,   // 0101
    125.0f,   // 0110
    62.5f,    // 0111
    31.25f,   // 1000
};

// Rev A Table 22 column "ODR Rate (6DOF)", and Table 23 for the gyro. The two
// are the same ladder: with the gyro running, the accelerometer is retimed onto
// the gyro's oscillator.
constexpr float kSixDofOdr[9] = {
    7174.4f, 3587.2f, 1793.6f, 896.8f, 448.4f, 224.2f, 112.1f, 56.05f, 28.025f,
};

// Low-power codes 1100..1111, accel-only. Rev A marks all four N/A in 6DOF,
// which is the register-level reason WoM forces the gyro off.
constexpr float kLowPowerOdr[4] = {128.0f, 21.0f, 11.0f, 3.0f};

}  // namespace

// -------------------------------------------------------------- free functions

float accelOdrHzFor(uint8_t aOdrCode, bool gyroEnabled)
{
    const uint8_t code = aOdrCode & 0x0F;

    if (code >= 0x0C) {
        // Low power exists only while the gyro is off.
        return gyroEnabled ? 0.0f : kLowPowerOdr[code - 0x0C];
    }
    if (code > 0x08) {
        return 0.0f;  // 1001..1011 are N/A in both columns
    }
    return gyroEnabled ? kSixDofOdr[code] : kAccelOnlyOdr[code];
}

float gyroOdrHzFor(uint8_t gOdrCode)
{
    const uint8_t code = gOdrCode & 0x0F;
    return (code <= 0x08) ? kSixDofOdr[code] : 0.0f;
}

// ---------------------------------------------------------------------- Qmi8658

Qmi8658::Qmi8658(i2c::Bus& bus, DelayFn delay) : bus_(bus), delay_(delay) {}

bool Qmi8658::readReg(uint8_t reg, uint8_t& value)
{
    return bus_.readRegs(reg, &value, 1);
}

bool Qmi8658::updateReg(uint8_t reg, uint8_t clearMask, uint8_t setMask)
{
    // Read-modify-write is TWO transactions and the bus lock is released
    // between them. Safe only because one task owns this device -- the rule
    // i2c::Device enforces at runtime. Section 7.1 of the bus design.
    uint8_t v = 0;
    if (!readReg(reg, v)) {
        return false;
    }
    const uint8_t next = static_cast<uint8_t>((v & ~clearMask) | setMask);
    return (next == v) ? true : bus_.writeReg(reg, next);
}

// --------------------------------------------------------------- identification

bool Qmi8658::readRegister(uint8_t reg, uint8_t& value)
{
    return bus_.readRegs(reg, &value, 1);
}

bool Qmi8658::writeRegister(uint8_t reg, uint8_t value)
{
    return bus_.writeReg(reg, value);
}

bool Qmi8658::burstRead(uint8_t reg, uint8_t* dst, std::size_t len)
{
    const uint8_t addr =
        (burstHighBit_ && len > 1) ? static_cast<uint8_t>(reg | 0x80) : reg;
    return bus_.readRegs(addr, dst, len);
}

bool Qmi8658::burstWrite(uint8_t reg, const uint8_t* src, std::size_t len)
{
    const uint8_t addr =
        (burstHighBit_ && len > 1) ? static_cast<uint8_t>(reg | 0x80) : reg;
    return bus_.writeRegs(addr, src, len);
}

bool Qmi8658::identify(IdentityReport& r)
{
    r = IdentityReport{};

    // 1+2. One byte at a time. These need no auto-increment at all, so they
    //      answer "is the chip there" without depending on the very feature
    //      that might be broken.
    if (!readRegister(reg::WHO_AM_I, r.whoAmI) ||
        !readRegister(reg::REVISION_ID, r.revision)) {
        return false;  // the bus itself did not answer
    }
    r.read      = true;
    r.isQmi8658 = (r.whoAmI == reg::WHO_AM_I_VALUE);
    if (!r.isQmi8658) {
        return false;
    }

    // 3. Does the register pointer advance? Compare against what step 2
    //    actually read, not against a hard coded revision: a part with a
    //    different revision byte is still a working part.
    burstHighBit_ = false;
    if (bus_.readRegs(reg::WHO_AM_I, r.burst, 2) &&
        r.burst[0] == r.whoAmI && r.burst[1] == r.revision) {
        r.autoIncrement = true;
        return true;
    }

    // 4. It did not. Try the other convention the two datasheet revisions
    //    disagree about: bit 7 of the register address as the
    //    auto-increment enable. If that works, remember it -- every
    //    multi-byte access from here on uses it.
    uint8_t alt[2] = {0, 0};
    if (bus_.readRegs(static_cast<uint8_t>(reg::WHO_AM_I | 0x80), alt, 2) &&
        alt[0] == r.whoAmI && alt[1] == r.revision) {
        r.burst[0]           = alt[0];
        r.burst[1]           = alt[1];
        r.autoIncrement      = true;
        r.needsHighBitBurst  = true;
        burstHighBit_        = true;
        return true;
    }

    return false;  // chip is there, but no burst read works
}

bool Qmi8658::probe()
{
    uint8_t who = 0;
    return readRegister(reg::WHO_AM_I, who) && who == reg::WHO_AM_I_VALUE;
}

bool Qmi8658::beginReset()
{
    return bus_.writeReg(reg::RESET, reg::RESET_COMMAND);
}

bool Qmi8658::resetSucceeded()
{
    uint8_t v = 0;
    // Only valid right now: enabling a sensor or running any CTRL9 command
    // overwrites this register.
    return readReg(reg::RESET_RESULT, v) && v == reg::RESET_RESULT_VALUE;
}

// ----------------------------------------------------------------- board setup

bool Qmi8658::setAddressAutoIncrement(bool on)
{
    return on ? updateReg(reg::CTRL1, 0, reg::ctrl1::ADDR_AI)
              : updateReg(reg::CTRL1, reg::ctrl1::ADDR_AI, 0);
}

bool Qmi8658::setCtrl9Handshake(Ctrl9Handshake mode)
{
    return (mode == Ctrl9Handshake::StatusRegister)
               ? updateReg(reg::CTRL8, 0, reg::ctrl8::HANDSHAKE_VIA_STATUS)
               : updateReg(reg::CTRL8, reg::ctrl8::HANDSHAKE_VIA_STATUS, 0);
}

bool Qmi8658::setActivityIntPin(IntPin pin)
{
    return (pin == IntPin::Int1)
               ? updateReg(reg::CTRL8, 0, reg::ctrl8::ACTIVITY_INT_SEL_INT1)
               : updateReg(reg::CTRL8, reg::ctrl8::ACTIVITY_INT_SEL_INT1, 0);
}

// ------------------------------------------------------------- CTRL9 handshake

bool Qmi8658::ctrl9Command(uint8_t command)
{
    if (!bus_.writeReg(reg::CTRL9, command)) {
        return false;
    }

    // Poll STATUSINT.bit7. This works only because CTRL8.bit7 was set: the
    // default routes completion to INT1, which is not wired on this board, and
    // the wait would never end.
    bool done = false;
    for (uint32_t i = 0; i < kCtrl9MaxPolls; ++i) {
        uint8_t status = 0;
        if (!readReg(reg::STATUSINT, status)) {
            return false;
        }
        if (status & reg::statusint::CMD_DONE) {
            done = true;
            break;
        }
        if (delay_ != nullptr) {
            delay_(1);
        }
    }
    if (!done) {
        return false;  // bounded: never hang the task on a silent chip
    }

    // The ACK is not optional. Without it the device stays mid-protocol and the
    // NEXT CTRL9 command is ignored.
    return bus_.writeReg(reg::CTRL9, reg::cmd::ACK);
}

// ------------------------------------------------------------------ wake on motion

bool Qmi8658::configureWakeOnMotion()
{
    // Rev A section 9.4, in the documented order. Deviating from it is how you
    // get spurious wake-ups from the accelerometer's own start-up transient.

    // 1. every sensor off
    if (!bus_.writeReg(reg::CTRL7, 0x00)) {
        return false;
    }

    // 2. accelerometer full-scale + LOW POWER ODR. Low power is accel-only, so
    //    this step is also what makes WoM incompatible with the gyro.
    const uint8_t ctrl2 =
        static_cast<uint8_t>((static_cast<uint8_t>(accelRange_) << 4) | kAOdrLp11);
    if (!bus_.writeReg(reg::CTRL2, ctrl2)) {
        return false;
    }
    aOdrCode_    = kAOdrLp11;
    gyroEnabled_ = false;

    // 3. threshold in CAL1_L, interrupt selection + blanking in CAL1_H.
    //    CAL1_H[7:6]: 01 = INT2 (initial level 0), 00 = INT1 (initial level 0).
    const uint8_t sel   = (wom_.intPin == IntPin::Int2) ? 0x40 : 0x00;
    const uint8_t cal1H = static_cast<uint8_t>(sel | (wom_.blankingSamples & 0x3F));
    const uint8_t cal1[2] = {
        static_cast<uint8_t>(wom_.thresholdMg & 0xFF),  // 1 mg per LSB
        cal1H,
    };
    // One transaction for the pair: writeRegs exists for exactly this.
    if (!burstWrite(reg::CAL1_L, cal1, 2)) {
        return false;
    }

    // 4. hand the parameters to the device
    if (!ctrl9Command(reg::cmd::WRITE_WOM_SETTING)) {
        return false;
    }

    // 5. accelerometer back on -- and only now does WoM start watching
    if (!bus_.writeReg(reg::CTRL7, reg::ctrl7::AEN)) {
        return false;
    }
    womArmed_ = true;
    return true;
}

bool Qmi8658::disableWakeOnMotion()
{
    // Rev A section 12.6, in this exact order.
    if (!bus_.writeReg(reg::CTRL7, 0x00)) {
        return false;
    }

    // Threshold 0 is the documented "off" value. Without this the part
    // stays in WoM no matter what CTRL2 and CTRL7 are set to afterwards.
    const uint8_t cal1[2] = {0x00, 0x00};
    if (!burstWrite(reg::CAL1_L, cal1, 2)) {
        return false;
    }
    if (!ctrl9Command(reg::cmd::WRITE_WOM_SETTING)) {
        return false;
    }

    womArmed_ = false;
    return true;
}

// ------------------------------------------------------------------------ modes

void Qmi8658::setRanges(AccelRange accel, GyroRange gyro)
{
    accelRange_ = accel;
    gyroRange_  = gyro;
}

bool Qmi8658::applyMode(PowerMode mode, uint32_t& settleMsOut)
{
    settleMsOut = 0;

    if (mode == PowerMode::OFF) {
        // CTRL7 = 0, NOT CTRL1.sensorDisable: leaving power-down costs the
        // 1.75 s system turn-on time, which a watch cannot spend on a wrist
        // raise. Section 5.1.
        if (womArmed_ && !disableWakeOnMotion()) {
            return false;
        }
        if (!bus_.writeReg(reg::CTRL7, 0x00)) {
            return false;
        }
        gyroEnabled_ = false;
        mode_        = mode;
        return true;
    }

    if (mode == PowerMode::WOM) {
        if (!configureWakeOnMotion()) {
            return false;
        }
        mode_       = mode;
        settleMsOut = kAccelWakeMs + static_cast<uint32_t>(3000.0f / accelOdrHz());
        return true;
    }

    const bool wantGyro = (mode == PowerMode::IMU6);

    // Leaving WoM takes a documented exit sequence, not just new register
    // values. Skip it and the chip keeps the WoM behaviour while every
    // register reads back as the mode we asked for.
    if (womArmed_ && !disableWakeOnMotion()) {
        return false;
    }

    // Configure while the sensors are off, then enable. The reverse order lets
    // the first samples come out of a filter that is still settling.
    if (!bus_.writeReg(reg::CTRL7, 0x00)) {
        return false;
    }

    const uint8_t aCode = wantGyro ? kAOdr6Dof : kAOdr62_5;
    const uint8_t ctrl2 =
        static_cast<uint8_t>((static_cast<uint8_t>(accelRange_) << 4) | aCode);
    if (!bus_.writeReg(reg::CTRL2, ctrl2)) {
        return false;
    }

    if (wantGyro) {
        const uint8_t ctrl3 = static_cast<uint8_t>(
            (static_cast<uint8_t>(gyroRange_) << 4) | kGOdr112_1);
        if (!bus_.writeReg(reg::CTRL3, ctrl3)) {
            return false;
        }
        gOdrCode_ = kGOdr112_1;
    }

    uint8_t enables = reg::ctrl7::AEN;
    if (wantGyro) {
        enables |= reg::ctrl7::GEN;
    }
    if (!bus_.writeReg(reg::CTRL7, enables)) {
        return false;
    }

    aOdrCode_    = aCode;
    gyroEnabled_ = wantGyro;
    mode_        = mode;

    const float    odr  = accelOdrHz();
    const uint32_t wake = wantGyro ? kGyroWakeMs : kAccelWakeMs;
    settleMsOut = wake + ((odr > 0.0f) ? static_cast<uint32_t>(3000.0f / odr) : 0u);
    return true;
}

float Qmi8658::accelOdrHz() const
{
    return accelOdrHzFor(aOdrCode_, gyroEnabled_);
}

float Qmi8658::gyroOdrHz() const
{
    return gyroEnabled_ ? gyroOdrHzFor(gOdrCode_) : 0.0f;
}

// ---------------------------------------------------------------------- reading

bool Qmi8658::readSample(RawSample& out)
{
    uint8_t buf[12];
    if (!burstRead(reg::AX_L, buf, sizeof(buf))) {
        return false;
    }
    // Little endian: low byte first, matching the L/H register naming. CTRL1.BE
    // defaults to 1 and claims otherwise, but SensorLib reads it this way and
    // works on this silicon. If the numbers ever look absurd, this line and that
    // bit are the first place to look. Section 19.
    for (int i = 0; i < 3; ++i) {
        out.accel[i] = static_cast<int16_t>(
            static_cast<uint16_t>(buf[2 * i]) |
            (static_cast<uint16_t>(buf[2 * i + 1]) << 8));
    }
    for (int i = 0; i < 3; ++i) {
        out.gyro[i] = static_cast<int16_t>(
            static_cast<uint16_t>(buf[6 + 2 * i]) |
            (static_cast<uint16_t>(buf[6 + 2 * i + 1]) << 8));
    }
    return true;
}

bool Qmi8658::readTemperature(int16_t& raw)
{
    uint8_t buf[2];
    if (!burstRead(reg::TEMP_L, buf, sizeof(buf))) {
        return false;
    }
    raw = static_cast<int16_t>(static_cast<uint16_t>(buf[0]) |
                               (static_cast<uint16_t>(buf[1]) << 8));
    return true;
}

bool Qmi8658::readStatus(StatusFlags& out)
{
    // STATUS0 and STATUS1 are adjacent, so one burst gets both -- and reading
    // STATUS1 CLEARS its flags, which is precisely why this must be a single
    // call that decodes everything at once rather than several targeted reads.
    uint8_t s[2] = {0, 0};
    if (!burstRead(reg::STATUS0, s, 2)) {
        return false;
    }

    out.accelReady   = (s[0] & reg::status0::ADA) != 0;
    out.gyroReady    = (s[0] & reg::status0::GDA) != 0;
    out.tap          = (s[1] & reg::status1::TAP) != 0;
    out.wakeOnMotion = (s[1] & reg::status1::WOM) != 0;
    out.step         = (s[1] & reg::status1::PEDOMETER) != 0;
    out.anyMotion    = (s[1] & reg::status1::ANY_MOTION) != 0;
    out.noMotion     = (s[1] & reg::status1::NO_MOTION) != 0;
    out.sigMotion    = (s[1] & reg::status1::SIG_MOTION) != 0;
    return true;
}

bool Qmi8658::readStepCount(uint32_t& steps)
{
    uint8_t buf[3];
    if (!burstRead(reg::STEP_CNT_LOW, buf, sizeof(buf))) {
        return false;
    }
    steps = static_cast<uint32_t>(buf[0]) |
            (static_cast<uint32_t>(buf[1]) << 8) |
            (static_cast<uint32_t>(buf[2]) << 16);
    return true;
}

}  // namespace imu
