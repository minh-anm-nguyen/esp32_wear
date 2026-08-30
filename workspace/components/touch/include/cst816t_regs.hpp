// CST816T register map. Constants only, no logic, no includes beyond <cstdint>.
//
// PROVENANCE, because the wrong document silently produces the wrong chip --
// the exact trap qmi8658_regs.hpp documents for Rev 0.6 vs Rev A:
//
//   Registers      "CST816T SDK register description v1.3", 2023-05-18
//                  -> doc-design/CST_816_T_v1_3_b7bcb3b8f4.pdf
//   Electricals    "CST816S Datasheet Rev V1.4" (Waveshare)
//                  -> doc-design/CST816S_Datasheet_EN.pdf
//   S-only regs    "CST816S register declaration"
//                  -> doc-design/CST816S_register_declaration.pdf
//
// The T document is SHORTER than the S one, and that is the point. Everything
// the S declaration lists but the T document does not is collected at the
// bottom under `s_only`, unused, so that nobody reaches for it by reflex.
//
// THREE registers differ between S and T in ways that would not raise an error:
//
//   0xED IrqPluseWidth   S: unit 0.1 ms, range 1..200, default 10  (= 1.0 ms)
//                        T: unit 1 ms,   range 1..5,   default 1   (= 1.0 ms)
//        Identical in real time, 10x apart as a register value. Writing the S
//        value 10 to a T asks for 10 ms, outside the range the T document gives.
//
//   0xEC MotionMask      S: [2] EnConLR  [1] EnConUD  [0] EnDClick
//                        T: [0] EnDClick only
//
//   0xFA IrqCtl          S: has [0] OnceWLP; T does not document that bit.
//
// The datasheet numbers that actually drive design decisions (sections 4.1 and
// 6 of the S datasheet, the only electrical document either part has):
//
//   dynamic mode   1.6 mA    scanning fast, reporting touches
//   standby mode   6.0 uA    slow scan, still wakes the host on a finger
//   sleep mode     1.0 uA    entered by command, LEAVES ONLY BY RESET
//
//   "After not touching 2S, automatically enter standby mode."
//
// That 2 s is why the chip looks dead on the bus when idle, and the 267x gap
// between 1.6 mA and 6 uA is why DisAutoSleep must stay OFF on a watch. The
// architecture -- read only after an IRQ, never poll -- follows from those two
// numbers rather than from a preference.
#pragma once

#include <cstdint>

namespace touch {
namespace reg {

// ------------------------------------------------------------ touch data block

// Six CONTIGUOUS bytes, which is the whole reason a frame can be read
// atomically. Reading X and Y as separate transactions lets the finger move
// between them and lets the response window close in the middle.
inline constexpr uint8_t GESTURE_ID = 0x01;
inline constexpr uint8_t FINGER_NUM = 0x02;
inline constexpr uint8_t XPOS_H     = 0x03;  // [7:6] event flag, [3:0] X[11:8]
inline constexpr uint8_t XPOS_L     = 0x04;
inline constexpr uint8_t YPOS_H     = 0x05;  // [3:0] Y[11:8]. NO event bits here
inline constexpr uint8_t YPOS_L     = 0x06;

inline constexpr uint8_t FRAME_BASE = GESTURE_ID;
inline constexpr uint8_t FRAME_LEN  = 6;

// 0x00 is NOT documented for either part. Several third-party drivers read 7
// bytes from 0x00 anyway; this one does not, so that every byte it interprets
// has a line in a document.

// XposH bit layout.
inline constexpr uint8_t EVENT_SHIFT  = 6;
inline constexpr uint8_t EVENT_MASK   = 0xC0;
inline constexpr uint8_t COORD_H_MASK = 0x0F;  // [3:0]; [5:4] are undocumented

// -------------------------------------------------------------------- identity

// Four CONTIGUOUS bytes: one burst read identifies the part completely.
inline constexpr uint8_t CHIP_ID    = 0xA7;
inline constexpr uint8_t PROJ_ID    = 0xA8;
inline constexpr uint8_t FW_VERSION = 0xA9;
inline constexpr uint8_t FACTORY_ID = 0xAA;  // T only; the S declaration stops at 0xA9

inline constexpr uint8_t IDENT_BASE = CHIP_ID;
inline constexpr uint8_t IDENT_LEN  = 4;

// STILL NOT A REQUIRED VALUE. Neither PDF states an expected ChipID, and
// Espressif notes parts that fail identification outright while touch works.
// Identification REPORTS what it read and never gates init -- see
// Cst816t::identify().
//
// What follows is a MEASUREMENT, not a specification: read off
// ESP32-S3-Touch-LCD-1.69 V2.1 on 2026-08-30.
//
//     ChipID 0xB5   ProjID 0x23   FwVersion 0x01   FactoryID 0xFF
//
// Useful only for comparison when a log arrives from another board: a
// different value is worth noticing, never worth refusing to run over. One
// board is not a batch, and it is certainly not a datasheet.
inline constexpr uint8_t CHIP_ID_SEEN_ON_V2_1 = 0xB5;

// ------------------------------------------------------------------ power mode

inline constexpr uint8_t SLEEP_MODE = 0xE5;
// Datasheet 4.1: "In this mode the touch chip is in a deep sleep state ... and
// can be switched to the dynamic mode by the reset pin." There is no command
// that leaves this state. Writing it without owning the reset line strands the
// touch panel until the next power cycle.
inline constexpr uint8_t SLEEP_MODE_ENTER = 0x03;

// ------------------------------------------------------------ error auto-reset

// The chip resetting ITSELF is invisible to us except that its configuration is
// gone afterwards. All three bits default off and stay off in V1.
inline constexpr uint8_t ERR_RESET_CTL       = 0xEA;
inline constexpr uint8_t ERR_RESET_LONGPRESS = 1u << 2;  // EnLTRst
inline constexpr uint8_t ERR_RESET_LARGEAREA = 1u << 1;  // EnFatRst
inline constexpr uint8_t ERR_RESET_TWOFINGER = 1u << 0;  // En2FRst

// ------------------------------------------------------------- gesture enables

inline constexpr uint8_t MOTION_MASK      = 0xEC;
inline constexpr uint8_t MOTION_EN_DCLICK = 1u << 0;  // the only bit the T has

// ----------------------------------------------------------- interrupt config

inline constexpr uint8_t IRQ_PULSE_WIDTH  = 0xED;  // T: unit 1 ms, 1..5, default 1
inline constexpr uint8_t IRQ_PULSE_MIN_MS = 1;
inline constexpr uint8_t IRQ_PULSE_MAX_MS = 5;

inline constexpr uint8_t IRQ_CTL       = 0xFA;
inline constexpr uint8_t IRQ_EN_TEST   = 1u << 7;  // self-pulses, bring-up only
inline constexpr uint8_t IRQ_EN_TOUCH  = 1u << 6;  // periodic pulse while touched
inline constexpr uint8_t IRQ_EN_CHANGE = 1u << 5;  // pulse when state changes
inline constexpr uint8_t IRQ_EN_MOTION = 1u << 4;  // pulse on gesture

// ---------------------------------------------------------------- auto standby

inline constexpr uint8_t DIS_AUTO_SLEEP = 0xFE;
// "0 by default, enable automatic entry into low-power mode. When non-zero
// (< 0xF0), disable automatic entry into low-power mode."
//
// The >= 0xF0 exclusion is stated only in the T document. Any value written
// here stays well clear of it.
inline constexpr uint8_t AUTO_SLEEP_ENABLED   = 0x00;
inline constexpr uint8_t AUTO_SLEEP_DISABLED  = 0x01;
inline constexpr uint8_t AUTO_SLEEP_MAX_VALID = 0xEF;

// --------------------------------------------------------------- gesture codes

inline constexpr uint8_t GESTURE_NONE         = 0x00;
inline constexpr uint8_t GESTURE_SLIDE_UP     = 0x01;
inline constexpr uint8_t GESTURE_SLIDE_DOWN   = 0x02;
inline constexpr uint8_t GESTURE_SLIDE_LEFT   = 0x03;
inline constexpr uint8_t GESTURE_SLIDE_RIGHT  = 0x04;
inline constexpr uint8_t GESTURE_SINGLE_CLICK = 0x05;
inline constexpr uint8_t GESTURE_DOUBLE_CLICK = 0x0B;
inline constexpr uint8_t GESTURE_LONG_PRESS   = 0x0C;
inline constexpr uint8_t GESTURE_BIG_PALM     = 0xAA;  // T only

// ---------------------------------------------------------------------- s_only
//
// Present in the CST816S register declaration, ABSENT from the CST816T v1.3
// document. Listed so a future reader can see they were considered and
// rejected, not overlooked. Nothing in this component writes them.
//
// Losing AutoSleepTime hurts most: on a T the 2 s standby delay cannot be
// tuned, only disabled wholesale through DIS_AUTO_SLEEP -- and section 6 of
// the datasheet says that costs 1.6 mA instead of 6 uA.
namespace s_only {

inline constexpr uint8_t BPC0H            = 0xB0;
inline constexpr uint8_t BPC0L            = 0xB1;
inline constexpr uint8_t BPC1H            = 0xB2;
inline constexpr uint8_t BPC1L            = 0xB3;
inline constexpr uint8_t NOR_SCAN_PER     = 0xEE;
inline constexpr uint8_t MOTION_SL_ANGLE  = 0xEF;
inline constexpr uint8_t LP_SCAN_RAW1H    = 0xF0;
inline constexpr uint8_t LP_SCAN_RAW1L    = 0xF1;
inline constexpr uint8_t LP_SCAN_RAW2H    = 0xF2;
inline constexpr uint8_t LP_SCAN_RAW2L    = 0xF3;
inline constexpr uint8_t LP_AUTO_WAKETIME = 0xF4;
inline constexpr uint8_t LP_SCAN_TH       = 0xF5;
inline constexpr uint8_t LP_SCAN_WIN      = 0xF6;
inline constexpr uint8_t LP_SCAN_FREQ     = 0xF7;
inline constexpr uint8_t LP_SCAN_IDAC     = 0xF8;
inline constexpr uint8_t AUTO_SLEEP_TIME  = 0xF9;  // S: 1 s units, default 2 s
inline constexpr uint8_t AUTO_RESET       = 0xFB;
inline constexpr uint8_t LONG_PRESS_TIME  = 0xFC;
inline constexpr uint8_t IO_CTL           = 0xFD;  // SOFT_RST / IIC_OD / En1v8

}  // namespace s_only

}  // namespace reg
}  // namespace touch
