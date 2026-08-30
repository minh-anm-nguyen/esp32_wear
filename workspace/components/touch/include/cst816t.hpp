// Pure logic layer for the CST816T.
//
// Includes nothing from ESP-IDF. Its only window on the world is an i2c::Bus&,
// which is what lets frame parsing, the configuration write ORDER and the
// read-back verification all be tested on a PC against i2c::FakeBus, with no
// chip -- the same arrangement as qmi8658.hpp.
//
// It also has NO way to sleep and no way to touch a GPIO. Reset is a pin, so
// reset belongs to TouchManager; this class only says how long to hold it and
// what to do afterwards. That keeps the layer free of FreeRTOS and makes the
// timings assertable values instead of buried delays.
//
// THE ONE THING TO UNDERSTAND ABOUT THIS CHIP
//
// Datasheet 4.1: after 2 s without a touch the chip leaves dynamic mode for
// standby. In standby it still scans and still pulses IRQ when a finger
// arrives, but whether it answers I2C is not stated by any document we have.
// So this driver assumes it does not, and every write it performs is confined
// to the window right after a reset, when the chip is provably in dynamic
// mode. Reads happen only after an IRQ, when the chip has just proven it is
// awake by pulsing the line.
//
// Everything marked UNVERIFIED below is a guess with a number attached, chosen
// so a log can disprove it. See doc-design/CST816T-technical-challenges.md.
#pragma once

#include "cst816t_regs.hpp"

#include "i2c_bus.hpp"

#include <cstddef>
#include <cstdint>

namespace touch {

// ------------------------------------------------------------------ data types

// XposH[7:6]. The T document is explicit: 00 down, 01 up, 10 contact/move,
// 11 reserved.
enum class EventFlag : uint8_t {
    Down     = 0,
    Up       = 1,
    Contact  = 2,
    Reserved = 3,
};

// How a six-byte frame was judged. A frame that arrived intact over I2C but
// carries nonsense is NOT the same thing as a bus error, and neither is the
// same as a legitimate "no finger" -- conflating the three is what turns a
// corrupted read into a phantom tap at the edge of the screen.
enum class FrameStatus : uint8_t {
    Valid,           // one finger, coordinates inside the panel
    NoFinger,        // finger count 0: a release candidate, not an error
    AllOnes,         // every byte 0xFF: nobody is driving the bus
    BadFingerCount,  // > 1 on a single-touch part
    ReservedEvent,   // event flag 11
    OutOfRange,      // coordinate past the panel by more than the edge tolerance
};

const char* toString(FrameStatus s);
const char* toString(EventFlag e);

// The panel this driver is pointed at, in the coordinate frame the CHIP
// reports -- not the one the user sees. Transform is a separate concern; see
// touch_transform.hpp.
struct RawLimits {
    uint16_t width{240};
    uint16_t height{280};

    // A coordinate exactly `edgeTolerance` past the last pixel is pulled back
    // to it and counted, rather than rejected.
    //
    // UNVERIFIED, and deliberately 1 rather than 0. Capacitive panels commonly
    // report the far edge as width instead of width-1, and rejecting those
    // makes the right-hand column of the UI unreachable. Anything further out
    // is a corrupt frame and stays rejected: clamping a wild value would
    // manufacture a valid-looking press at the screen edge.
    uint16_t edgeTolerance{1};
};

// One parsed frame. Plain data, no methods, so a test can build one by hand.
struct ParsedFrame {
    FrameStatus status{FrameStatus::NoFinger};
    uint8_t     fingers{0};
    EventFlag   event{EventFlag::Up};
    uint16_t    rawX{0};
    uint16_t    rawY{0};
    uint8_t     gestureCode{reg::GESTURE_NONE};

    // The gesture byte held a code neither document lists. Not a frame error:
    // the coordinates in the same frame may be perfectly good.
    bool gestureUnknown{false};

    // A coordinate was pulled back by at most edgeTolerance.
    bool edgeClamped{false};

    // finger count and event flag disagreed about whether a finger is present
    // (count 0 with event Down, or count 1 with event Up). Both readings are
    // reported as-is; this flag exists so the manager can count how often the
    // chip does it, because which one to trust is a hardware question.
    bool fingerEventMismatch{false};

    bool pressed() const { return status == FrameStatus::Valid; }
};

// Pure function: six bytes in, a verdict out. No bus, no clock, no state --
// which is what makes the whole validation policy testable on a laptop.
ParsedFrame parseFrame(const uint8_t* frame, std::size_t len, const RawLimits& limits);

const char* gestureName(uint8_t code);

// ------------------------------------------------------------------- identity

// What the chip says it is. Reported, never enforced: no document we have
// states an expected ChipID, and Espressif records parts that fail this read
// while touch works fine.
struct ChipInfo {
    bool    read{false};   // the burst read itself succeeded
    uint8_t chipId{0};
    uint8_t projId{0};
    uint8_t fwVersion{0};
    uint8_t factoryId{0};

    // Every byte came back 0xFF or every byte came back 0x00. Both mean the
    // read produced nothing trustworthy even though I2C acknowledged.
    bool implausible() const
    {
        const bool allFf = chipId == 0xFF && projId == 0xFF && fwVersion == 0xFF
                        && factoryId == 0xFF;
        const bool allZero = chipId == 0 && projId == 0 && fwVersion == 0 && factoryId == 0;
        return allFf || allZero;
    }
};

// -------------------------------------------------------------- configuration

// What to write into the chip during the post-reset window.
//
// Ordering matters and is handled by applyConfig(): IRQ_CTL is written LAST so
// the chip does not start pulsing the line before the rest of its behaviour is
// settled.
struct ChipConfig {
    // 0xFA. EnTouch gives a periodic pulse for as long as a finger is down,
    // which is what feeds LVGL a drag; EnChange gives an edge when the state
    // changes, which is what makes the release prompt. Both on is the
    // combination that serves a UI, at the cost of more interrupts than
    // gesture-only mode.
    bool irqOnTouch{true};
    bool irqOnChange{true};

    // Hardware gesture recognition. OFF in V1: with the screen on, LVGL owns
    // gestures, and firing both produces one swipe handled twice.
    bool irqOnMotion{false};

    // 0xFA bit 7. Makes the chip pulse IRQ on its own with no finger anywhere.
    // Bring-up only -- it proves the INT wiring and the ISR path without
    // needing a human to touch the glass.
    bool irqSelfTest{false};

    // 0xED, in milliseconds on the T (1..5). The default 1 ms is enough for a
    // GPIO interrupt on a running CPU. It is NOT obviously enough for a
    // level-sampled wake from light sleep, which is why V2 has to revisit it.
    uint8_t irqPulseMs{reg::IRQ_PULSE_MIN_MS};

    // 0xEC bit 0. Off in V1 for the same reason as irqOnMotion.
    bool enableDoubleClick{false};

    // 0xFE. LEAVE FALSE ON A WATCH.
    //
    // Setting it stops the 2 s slide into standby, which makes the chip answer
    // I2C at any time and makes bring-up far easier -- at 1.6 mA instead of
    // 6 uA, per datasheet section 6. That is roughly a 267x idle-current
    // penalty and it runs 24 hours a day. Useful on the bench with USB
    // attached; never in a shipped power profile.
    bool disableAutoSleep{false};

    // 0xEA. All off: a chip that resets itself silently drops the
    // configuration written here, and the manager would only find out by
    // noticing the behaviour changed.
    bool resetOnLongPress{false};
    bool resetOnLargeArea{false};
    bool resetOnTwoFinger{false};

    // Read every register back after writing it. Cheap (five extra one-byte
    // reads, once, inside the post-reset window) and the only evidence that
    // the chip accepted what it was told.
    bool verifyWrites{true};
};

// Per-register outcome of applyConfig(). Not a bool, because "the chip refused
// register 0xED" and "the bus is dead" call for completely different responses,
// and because a log that names the register is worth ten that say "config
// failed".
struct ConfigResult {
    static constexpr std::size_t kMaxEntries = 5;

    struct Entry {
        uint8_t regAddr{0};
        uint8_t wrote{0};
        uint8_t readBack{0};
        bool    writeOk{false};
        bool    verified{false};  // false also when verifyWrites was off
    };

    Entry       entries[kMaxEntries]{};
    std::size_t count{0};

    bool allWritesOk() const
    {
        for (std::size_t i = 0; i < count; ++i) {
            if (!entries[i].writeOk) return false;
        }
        return count > 0;
    }

    bool allVerified() const
    {
        for (std::size_t i = 0; i < count; ++i) {
            if (!entries[i].verified) return false;
        }
        return count > 0;
    }

    // First register that did not read back what was written, or 0 if none.
    uint8_t firstMismatch() const
    {
        for (std::size_t i = 0; i < count; ++i) {
            if (!entries[i].verified) return entries[i].regAddr;
        }
        return 0;
    }
};

// ------------------------------------------------------------ reset timing

// Reset behaviour that no document we have specifies, so every value here is a
// starting point chosen to be safely long, not a measured one.
//
// What IS documented (datasheet 4.3): RST is active LOW, and the pin has a
// built-in pull-up and an RC filter. The RC filter is precisely why a very
// short pulse cannot be trusted -- it is a low-pass, and the datasheet gives
// neither its corner frequency nor a minimum pulse width.
struct ResetTiming {
    // UNVERIFIED. 10 ms is far longer than any RC filter on a 3.3 V CMOS input
    // is likely to need, and costs nothing at boot.
    uint32_t assertMs{10};

    // UNVERIFIED. Time from releasing RST until the chip answers I2C. Reported
    // ranges for this family run from 5 ms to 50 ms; 50 ms is the safe end and
    // is paid once per reset.
    uint32_t settleMs{50};

    // How long after a reset the chip is certainly still in dynamic mode and
    // will certainly answer. Datasheet 4.1 says standby follows 2 s without a
    // touch, so all configuration must land inside this.
    //
    // 1500 ms rather than 2000: the margin absorbs a task preemption between
    // the reset and the last register write.
    uint32_t configWindowMs{1500};
};

// ----------------------------------------------------------------------- Cst816t

class Cst816t {
public:
    explicit Cst816t(i2c::Bus& bus) : bus_(bus) {}

    // One burst read of the four contiguous identity registers.
    //
    // NEVER gates anything. Returns what it saw so the caller can log it; a
    // chip that refuses this read but reports coordinates is a working touch
    // panel, and treating identification as mandatory is exactly how a driver
    // bricks itself on a batch it has never met.
    ChipInfo identify();

    // Writes the whole configuration, IRQ_CTL last, verifying as it goes.
    //
    // MUST be called inside the post-reset window: see ResetTiming. Calling it
    // on an idle chip that has slid into standby is expected to NACK, and that
    // is not evidence of a fault.
    ConfigResult applyConfig(const ChipConfig& config);

    // One atomic six-byte burst from 0x01, then parse.
    //
    // `busOk` distinguishes "the transaction failed" from "the transaction
    // succeeded and the contents are nonsense". They look identical if the
    // return value is only the frame.
    ParsedFrame readFrame(const RawLimits& limits, bool& busOk);

    // Same read, but hands back the raw bytes as well, for the bring-up log.
    ParsedFrame readFrame(const RawLimits& limits, bool& busOk, uint8_t* rawOut);

    // Writes 0xE5 = 0x03. THE CHIP DOES NOT COME BACK FROM THIS WITHOUT A
    // RESET PULSE -- datasheet 4.1. Do not call it unless the caller owns the
    // reset line and intends to use it.
    bool enterDeepSleep();

    // The byte applyConfig() would write to IRQ_CTL. Exposed because the
    // manager logs it, and because a test can assert the mapping without
    // reaching into the bus log.
    static uint8_t irqCtlByte(const ChipConfig& config);
    static uint8_t motionMaskByte(const ChipConfig& config);
    static uint8_t errResetByte(const ChipConfig& config);
    static uint8_t autoSleepByte(const ChipConfig& config);
    static uint8_t irqPulseByte(const ChipConfig& config);

private:
    bool writeVerified(ConfigResult& out, uint8_t regAddr, uint8_t value, bool verify);

    i2c::Bus& bus_;
};

}  // namespace touch
