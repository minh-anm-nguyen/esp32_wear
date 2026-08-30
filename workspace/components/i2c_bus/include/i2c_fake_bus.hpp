// A 256-byte register file that pretends to be an I2C device, plus a log of
// every transaction and a fault injector.
//
// Header only and allocation free, so it drops into a host test with nothing
// but an include. It is what lets a driver's CTRL9 handshake, its WoM setup
// sequence and its two-command pedometer load all be tested on a PC, in the
// right ORDER, without a chip.
//
// Shared by every driver's tests -- that is the reason it lives in i2c_bus and
// not in whichever component happened to need it first.
#pragma once

#include "i2c_bus.hpp"

namespace i2c {

// One completed transaction, as the wire saw it.
struct FakeTransaction {
    enum class Kind : uint8_t { Read, Write };

    Kind        kind{Kind::Read};
    uint8_t     reg{0};
    std::size_t len{0};

    bool operator==(const FakeTransaction& o) const
    {
        return kind == o.kind && reg == o.reg && len == o.len;
    }
};

class FakeBus : public Bus {
public:
    static constexpr std::size_t kLogCapacity = 64;

    // ------------------------------------------------------------ Bus contract

    bool readRegs(uint8_t reg, uint8_t* dst, std::size_t len) override
    {
        if (!record(FakeTransaction::Kind::Read, reg, len)) {
            return false;
        }
        for (std::size_t i = 0; i < len; ++i) {
            dst[i] = regs_[static_cast<uint8_t>(reg + i)];  // wraps, like the chip
        }
        return true;
    }

    bool writeReg(uint8_t reg, uint8_t value) override
    {
        return writeRegs(reg, &value, 1);
    }

    bool writeRegs(uint8_t reg, const uint8_t* src, std::size_t len) override
    {
        if (!record(FakeTransaction::Kind::Write, reg, len)) {
            return false;
        }
        for (std::size_t i = 0; i < len; ++i) {
            regs_[static_cast<uint8_t>(reg + i)] = src[i];
        }
        return true;
    }

    // ------------------------------------------------------ test-side controls

    void    setReg(uint8_t reg, uint8_t value) { regs_[reg] = value; }
    uint8_t getReg(uint8_t reg) const { return regs_[reg]; }

    // Make the next n transactions fail, as a NACK would. The register file is
    // left untouched on a failed write, which is the honest thing: a NACKed
    // write did not land.
    void failNext(uint32_t n) { failCountdown_ = n; }

    // Transactions since the last clearLog(), oldest first. Capped; overflowed()
    // tells you the log stopped being complete.
    const FakeTransaction* log() const { return log_; }
    std::size_t            logCount() const { return logCount_; }
    bool                   logOverflowed() const { return logOverflowed_; }

    void clearLog()
    {
        logCount_      = 0;
        logOverflowed_ = false;
    }

    void reset()
    {
        for (std::size_t i = 0; i < 256; ++i) {
            regs_[i] = 0;
        }
        clearLog();
        failCountdown_ = 0;
    }

private:
    bool record(FakeTransaction::Kind kind, uint8_t reg, std::size_t len)
    {
        if (failCountdown_ > 0) {
            --failCountdown_;
            return false;  // not logged: it never reached the device
        }
        if (logCount_ < kLogCapacity) {
            log_[logCount_++] = FakeTransaction{kind, reg, len};
        } else {
            logOverflowed_ = true;
        }
        return true;
    }

    uint8_t         regs_[256]{};
    FakeTransaction log_[kLogCapacity]{};
    std::size_t     logCount_{0};
    bool            logOverflowed_{false};
    uint32_t        failCountdown_{0};
};

}  // namespace i2c
