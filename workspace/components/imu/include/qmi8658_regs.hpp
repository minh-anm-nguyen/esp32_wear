// QMI8658C register map. Constants only, no logic, no includes beyond <cstdint>.
//
// EVERY value here comes from **Rev A (2022)**, document 13-52-27, which is the
// revision this silicon actually matches -- verified against SensorLib, whose
// constants agree with Rev A byte for byte.
//
// Do NOT cross-check against the widely mirrored Rev 0.6 (2021). It calls CTRL8
// "Reserved", tops the gyro out at +/-2048 dps, lists the gyro ODR as
// 31.25..8000 Hz, and -- worst of all -- gives RST_FIFO and REQ_FIFO the values
// 0x05 and 0x0D, where Rev A uses 0x04 and 0x05. The same byte 0x05 means
// "reset the FIFO" in one revision and "request it" in the other, with no error
// either way. See doc-design/imu-qmi8658c-design.md section 19.
#pragma once

#include <cstdint>

namespace imu {
namespace reg {

// -------------------------------------------------------------- general purpose

inline constexpr uint8_t WHO_AM_I     = 0x00;  // -> 0x05
inline constexpr uint8_t REVISION_ID  = 0x01;  // -> 0x68

inline constexpr uint8_t WHO_AM_I_VALUE    = 0x05;
inline constexpr uint8_t REVISION_ID_VALUE = 0x68;

// ------------------------------------------------------------ setup and control

inline constexpr uint8_t CTRL1 = 0x02;  // serial interface + oscillator
inline constexpr uint8_t CTRL2 = 0x03;  // accelerometer: self test, FS, ODR
inline constexpr uint8_t CTRL3 = 0x04;  // gyroscope:     self test, FS, ODR
inline constexpr uint8_t CTRL4 = 0x05;  // magnetometer (not fitted on this board)
inline constexpr uint8_t CTRL5 = 0x06;  // low-pass filters
inline constexpr uint8_t CTRL6 = 0x07;  // AttitudeEngine (deliberately unused)
inline constexpr uint8_t CTRL7 = 0x08;  // sensor enables
inline constexpr uint8_t CTRL8 = 0x09;  // motion detection control
inline constexpr uint8_t CTRL9 = 0x0A;  // host command register

// Parameter registers for the CTRL9 protocol.
inline constexpr uint8_t CAL1_L = 0x0B;
inline constexpr uint8_t CAL1_H = 0x0C;
inline constexpr uint8_t CAL2_L = 0x0D;
inline constexpr uint8_t CAL2_H = 0x0E;
inline constexpr uint8_t CAL3_L = 0x0F;
inline constexpr uint8_t CAL3_H = 0x10;
inline constexpr uint8_t CAL4_L = 0x11;
inline constexpr uint8_t CAL4_H = 0x12;

// ------------------------------------------------------------------------- FIFO

inline constexpr uint8_t FIFO_WTM_TH   = 0x13;
inline constexpr uint8_t FIFO_CTRL     = 0x14;
inline constexpr uint8_t FIFO_SMPL_CNT = 0x15;
inline constexpr uint8_t FIFO_STATUS   = 0x16;
inline constexpr uint8_t FIFO_DATA     = 0x17;

// ----------------------------------------------------------------------- status

inline constexpr uint8_t STATUSINT = 0x2D;
inline constexpr uint8_t STATUS0   = 0x2E;
inline constexpr uint8_t STATUS1   = 0x2F;

// ------------------------------------------------------------------------- data

inline constexpr uint8_t TIMESTAMP_LOW = 0x30;
inline constexpr uint8_t TEMP_L        = 0x33;
inline constexpr uint8_t TEMP_H        = 0x34;

inline constexpr uint8_t AX_L = 0x35;  // AX_L..GZ_H is 12 contiguous bytes,
inline constexpr uint8_t GZ_H = 0x40;  // read as ONE burst

inline constexpr uint8_t COD_STATUS = 0x46;  // 0x00 == calibration succeeded

// Gyro gains land here after a successful COD, for the host to save to NVS.
inline constexpr uint8_t DVX_L = 0x51;

inline constexpr uint8_t TAP_STATUS    = 0x59;
inline constexpr uint8_t STEP_CNT_LOW  = 0x5A;  // 24-bit, three bytes

// ------------------------------------------------------------------------ reset

inline constexpr uint8_t RESET = 0x60;

// Table 27 says write 0xB0. Section 7.4 of the SAME datasheet says 0x0B --
// transposed digits. SensorLib uses 0xB0 and works on this board, so 0xB0 it is.
inline constexpr uint8_t RESET_COMMAND = 0xB0;

// Valid only IMMEDIATELY after a reset: enabling a sensor or running any CTRL9
// command overwrites it.
inline constexpr uint8_t RESET_RESULT       = 0x4D;
inline constexpr uint8_t RESET_RESULT_VALUE = 0x80;

// ------------------------------------------------------------------- bit fields

namespace ctrl1 {
inline constexpr uint8_t SIM            = 1u << 7;
inline constexpr uint8_t ADDR_AI        = 1u << 6;  // MUST be set: see below
inline constexpr uint8_t BE             = 1u << 5;
inline constexpr uint8_t FIFO_INT_SEL   = 1u << 2;  // 0 = INT2
inline constexpr uint8_t SENSOR_DISABLE = 1u << 0;

// bits 4:3 are "Reserved" in Rev A, but SensorLib drives them as INT1/INT2
// output enables and works on this board. If GPIO38 stays silent with
// everything else configured correctly, set INT2_ENABLE. Section 19.
inline constexpr uint8_t INT1_ENABLE_UNDOCUMENTED = 1u << 3;
inline constexpr uint8_t INT2_ENABLE_UNDOCUMENTED = 1u << 4;
}  // namespace ctrl1

namespace ctrl7 {
inline constexpr uint8_t SYNC_SMPL = 1u << 7;
inline constexpr uint8_t SYS_HS    = 1u << 6;
inline constexpr uint8_t GSN       = 1u << 4;  // gyro snooze: drive on, sense off
inline constexpr uint8_t SEN       = 1u << 3;  // AttitudeEngine
inline constexpr uint8_t MEN       = 1u << 2;
inline constexpr uint8_t GEN       = 1u << 1;
inline constexpr uint8_t AEN       = 1u << 0;
}  // namespace ctrl7

namespace ctrl8 {
// 1 = poll STATUSINT.bit7 instead of waiting on INT1. Mandatory on this board:
// INT1 is not routed, so leaving this at 0 hangs every CTRL9 command forever.
inline constexpr uint8_t HANDSHAKE_VIA_STATUS = 1u << 7;

// 0 = INT2. Covers any/no/significant motion, pedometer AND tap, all at once.
inline constexpr uint8_t ACTIVITY_INT_SEL_INT1 = 1u << 6;

inline constexpr uint8_t PEDO_EN       = 1u << 4;
inline constexpr uint8_t SIG_MOTION_EN = 1u << 3;
inline constexpr uint8_t NO_MOTION_EN  = 1u << 2;
inline constexpr uint8_t ANY_MOTION_EN = 1u << 1;
inline constexpr uint8_t TAP_EN        = 1u << 0;
}  // namespace ctrl8

namespace statusint {
inline constexpr uint8_t CMD_DONE = 1u << 7;  // CTRL9 handshake
inline constexpr uint8_t LOCKED   = 1u << 1;
inline constexpr uint8_t AVAIL    = 1u << 0;
}  // namespace statusint

namespace status0 {
inline constexpr uint8_t ADA = 1u << 0;  // new accelerometer data
inline constexpr uint8_t GDA = 1u << 1;  // new gyroscope data
}  // namespace status0

// Reading STATUS1 CLEARS these flags. Read it once per interrupt and decode
// every bit from that one value -- a second read to ask a second question
// loses events. Section 3.2.
namespace status1 {
inline constexpr uint8_t CMD_DONE   = 1u << 0;
inline constexpr uint8_t TAP        = 1u << 1;
inline constexpr uint8_t WOM        = 1u << 2;
inline constexpr uint8_t PEDOMETER  = 1u << 4;
inline constexpr uint8_t ANY_MOTION = 1u << 5;
inline constexpr uint8_t NO_MOTION  = 1u << 6;
inline constexpr uint8_t SIG_MOTION = 1u << 7;
}  // namespace status1

// -------------------------------------------------------------- CTRL9 commands

namespace cmd {
inline constexpr uint8_t ACK                      = 0x00;
inline constexpr uint8_t RST_FIFO                 = 0x04;  // 0x05 in Rev 0.6!
inline constexpr uint8_t REQ_FIFO                 = 0x05;  // 0x0D in Rev 0.6!
inline constexpr uint8_t WRITE_WOM_SETTING        = 0x08;
inline constexpr uint8_t ACCEL_HOST_DELTA_OFFSET  = 0x09;
inline constexpr uint8_t GYRO_HOST_DELTA_OFFSET   = 0x0A;
inline constexpr uint8_t CONFIGURE_TAP            = 0x0C;
inline constexpr uint8_t CONFIGURE_PEDOMETER      = 0x0D;
inline constexpr uint8_t CONFIGURE_MOTION         = 0x0E;
inline constexpr uint8_t RESET_PEDOMETER          = 0x0F;
inline constexpr uint8_t COPY_USID                = 0x10;
inline constexpr uint8_t SET_RPU                  = 0x11;
inline constexpr uint8_t ON_DEMAND_CALIBRATION    = 0xA2;
inline constexpr uint8_t APPLY_GYRO_GAINS         = 0xAA;
}  // namespace cmd

}  // namespace reg
}  // namespace imu
