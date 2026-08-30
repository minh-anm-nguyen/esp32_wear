// Frames plus a clock in, Down/Move/Up out. Pure logic: no bus, no FreeRTOS,
// no LVGL, and no way to read the time -- `nowMs` is always a parameter, the
// same trick qmi8658.hpp uses to keep settling times assertable.
//
// WHY A BUFFER AND NOT JUST A "LATEST STATE"
//
// LVGL asks for the pointer state on a timer. A tap that goes down and up
// between two of those asks leaves a latest-state cache reading "released", and
// the tap simply never happened -- intermittently, depending on timing, which
// is the hardest class of bug to reproduce. So transitions are queued and
// consumed, while Move is allowed to collapse: the UI needs every Down and
// every Up, but only the newest position.
//
// LVGL 9 drains this correctly with `lv_indev_data_t.continue_reading`, which
// makes it call the read callback again in the same pass until the queue is
// empty. That is the whole reason the queue can stay this small.
#pragma once

#include "cst816t.hpp"
#include "touch_transform.hpp"

#include <cstdint>

namespace touch {

enum class TransitionKind : uint8_t {
    Down,
    Move,
    Up,
};

const char* toString(TransitionKind k);

struct TouchTransition {
    TransitionKind kind{TransitionKind::Up};
    int16_t        x{0};
    int16_t        y{0};
    uint32_t       timestampMs{0};

    // Monotonic. A gap means transitions were REALLY lost -- the queue
    // overflowed and had to evict. Collapsing a Move does not create a gap, so
    // an adapter that resets its state on a gap does not do it mid-drag.
    uint32_t sequence{0};

    // The chip did not say this happened -- a timeout or a fault did. Only
    // ever set on Up. Worth surfacing: a UI that sees many of these is a UI
    // whose releases are being guessed.
    bool synthetic{false};
};

// Read without consuming. Safe to poll from diagnostics or a status screen
// without stealing events from the UI.
struct TouchSnapshot {
    bool     pressed{false};
    int16_t  x{0};
    int16_t  y{0};
    uint32_t timestampMs{0};
    uint32_t sequence{0};
    uint8_t  lastGesture{reg::GESTURE_NONE};
};

struct TrackerConfig {
    // How long a contact may go unrefreshed before a release is invented.
    //
    // MEASURED on V2.1, 2026-08-30, with IRQ_EN_TOUCH | IRQ_EN_CHANGE: the gap
    // between interrupts during a contact was min 12 ms, mean 12 ms, max 14 ms
    // over 418 interrupts, including a finger held perfectly still (the chip
    // keeps pulsing at the same rate and simply repeats the coordinate). 60 ms
    // is a little over four times the worst case observed.
    //
    // It is a safety net and nothing more. Across that whole run every release
    // came from the chip itself -- syntheticUpCount stayed at zero -- so this
    // only ever fires when something has already gone wrong, and its value
    // decides how long a widget stays stuck when it does.
    //
    // Re-measure before trusting it in a different interrupt mode: gesture-only
    // operation does not pulse periodically at all, and this number would then
    // be describing a stream that no longer exists.
    uint32_t releaseTimeoutMs{60};

    // Believe an Up in the event flag even when the finger count still says 1.
    //
    // MEASURED on V2.1 (ChipID 0xB5, FW 0x01): the two never disagreed once in
    // 418 interrupts. This firmware releases with finger count 0 AND event Up
    // together, in the same frame, every time -- so on this part the setting
    // has no effect at all and has never fired.
    //
    // Left in, and left on. It costs nothing here and the disagreement is
    // reported on other CST816 firmware; if it ever happens the two failure
    // modes are not equal, since a missed Move costs one frame of a drag while
    // a missed Up leaves a widget held down until the next touch.
    // ParsedFrame::fingerEventMismatch keeps counting, so a future board that
    // does behave differently shows up in the diagnostics rather than as a
    // sticky button.
    bool trustEventUp{true};
};

// -------------------------------------------------------------- TouchTracker

class TouchTracker {
public:
    // Small on purpose. With Move collapsing, a drag occupies exactly one slot,
    // so this only has to survive a UI that stalls for several taps.
    static constexpr uint8_t kCapacity = 8;

    void configure(const TrackerConfig& config) { config_ = config; }
    const TrackerConfig& config() const { return config_; }

    // Feed one frame read after an IRQ, with its coordinate already
    // transformed. `point.valid` false means the frame carried no usable
    // position, which is not the same as the finger being gone.
    void onFrame(const ParsedFrame& frame, const TouchPoint& point, uint32_t nowMs);

    // Call when no frame arrived. This is what lets a release be synthesised
    // for a finger that lifted without the chip ever saying so.
    void tick(uint32_t nowMs);

    // Force a release. For the manager to call before it enters Fault, stops,
    // or hands the chip to a sleep mode -- LVGL must never be left holding a
    // widget down because the driver went away.
    void cancel(uint32_t nowMs);

    // Oldest queued transition. Returns false when empty.
    bool pop(TouchTransition& out);

    bool     empty() const { return count_ == 0; }
    uint8_t  queued() const { return count_; }

    TouchSnapshot snapshot() const { return snapshot_; }

    // ------------------------------------------------------------ diagnostics

    uint32_t downCount() const { return downCount_; }
    uint32_t upCount() const { return upCount_; }
    uint32_t moveCount() const { return moveCount_; }
    uint32_t syntheticUpCount() const { return syntheticUpCount_; }
    uint32_t coalescedMoveCount() const { return coalescedMoveCount_; }
    uint32_t overflowCount() const { return overflowCount_; }

    // Frames where the chip reported a finger but the event flag said Up.
    uint32_t eventUpWithFingerCount() const { return eventUpWithFingerCount_; }

    // Down arriving while already pressed: an Up went missing somewhere.
    uint32_t missedUpCount() const { return missedUpCount_; }

    void resetCounters();

private:
    void emit(TransitionKind kind, int16_t x, int16_t y, uint32_t nowMs, bool synthetic);
    void push(TouchTransition& t);
    void beginRelease(uint32_t nowMs, bool synthetic);

    TrackerConfig config_{};

    TouchTransition ring_[kCapacity]{};
    uint8_t         head_{0};
    uint8_t         count_{0};

    TouchSnapshot snapshot_{};
    uint32_t      sequence_{0};

    bool     pressed_{false};
    int16_t  lastX_{0};
    int16_t  lastY_{0};
    uint32_t lastContactMs_{0};

    uint32_t downCount_{0};
    uint32_t upCount_{0};
    uint32_t moveCount_{0};
    uint32_t syntheticUpCount_{0};
    uint32_t coalescedMoveCount_{0};
    uint32_t overflowCount_{0};
    uint32_t eventUpWithFingerCount_{0};
    uint32_t missedUpCount_{0};
};

}  // namespace touch
