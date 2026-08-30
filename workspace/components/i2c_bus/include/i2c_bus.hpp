// The seam every I2C device driver sees. Header only, includes nothing but
// <cstddef> and <cstdint>, so a driver's pure-logic layer stays compilable on a
// PC exactly as button.hpp and buzzer.hpp are.
//
// Three methods, deliberately. An earlier draft also had raw read()/write()
// without a register pointer; all three devices on this board (CST816T 0x15,
// PCF85063 0x51, QMI8658C 0x6B) are register-based, so nobody called them --
// yet every implementor, FakeBus included, still had to write them. That is
// Interface Segregation violated for nothing. If a device ever genuinely needs
// raw access, give it its own narrow interface instead of widening this one.
//
// See doc-design/i2c-bus-design.md section 6.1.
#pragma once

#include <cstddef>
#include <cstdint>

namespace i2c {

class Bus {
public:
    virtual ~Bus() = default;

    // Write the register pointer, then read -- as ONE transaction with a
    // repeated START. Never as a separate write followed by a read: the bus
    // lock is released between transactions, and another task's device could
    // slip in and move the shared hardware FSM. See design doc section 7.2.
    virtual bool readRegs(uint8_t reg, uint8_t* dst, std::size_t len) = 0;

    virtual bool writeReg(uint8_t reg, uint8_t value) = 0;

    // Consecutive registers, one transaction. Needed far more often than it
    // looks: PCF85063 takes 7 registers to set the time, QMI8658C takes 8
    // (CAL1_L..CAL4_H) to hand the pedometer its parameters.
    virtual bool writeRegs(uint8_t reg, const uint8_t* src, std::size_t len) = 0;
};

}  // namespace i2c
