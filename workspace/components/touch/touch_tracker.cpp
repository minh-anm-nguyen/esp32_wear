#include "touch_tracker.hpp"

namespace touch {

const char* toString(TransitionKind k)
{
    switch (k) {
    case TransitionKind::Down: return "Down";
    case TransitionKind::Move: return "Move";
    case TransitionKind::Up:   return "Up";
    }
    return "?";
}

// ---------------------------------------------------------------- queueing

void TouchTracker::push(TouchTransition& t)
{
    // Move collapses into a Move already at the tail. This is what keeps a
    // drag -- which produces a transition roughly every 10 ms -- from filling
    // the queue: the UI only ever needs the newest position, never the path
    // between two polls.
    if (t.kind == TransitionKind::Move && count_ > 0) {
        const uint8_t tail = static_cast<uint8_t>((head_ + count_ - 1) % kCapacity);
        if (ring_[tail].kind == TransitionKind::Move) {
            // KEEPS THE OLD SEQUENCE NUMBER. Collapsing loses no event, so the
            // consumer must not see a gap -- gaps are reserved for real loss
            // below. Burning a number here instead would make every drag look
            // like dropped events, and an adapter that resets on a gap would
            // reset several times a second while a finger is moving.
            t.sequence  = ring_[tail].sequence;
            ring_[tail] = t;
            ++coalescedMoveCount_;
            return;
        }
    }

    t.sequence = ++sequence_;

    if (count_ < kCapacity) {
        ring_[(head_ + count_) % kCapacity] = t;
        ++count_;
        return;
    }

    // Full. Something has to go, and the choice is not arbitrary:
    //
    //   dropping a Move   costs one frame of a drag
    //   dropping a Down   costs a tap
    //   dropping an Up    leaves the UI holding a widget down forever
    //
    // So evict the oldest Move. If there is none, evict the oldest Down --
    // losing a tap is bad, a stuck press is worse. An Up is never evicted.
    ++overflowCount_;

    for (uint8_t pass = 0; pass < 2; ++pass) {
        const TransitionKind victim = (pass == 0) ? TransitionKind::Move : TransitionKind::Down;
        for (uint8_t i = 0; i < count_; ++i) {
            const uint8_t idx = static_cast<uint8_t>((head_ + i) % kCapacity);
            if (ring_[idx].kind != victim) {
                continue;
            }
            // Close the gap by shifting the entries after it one step back.
            for (uint8_t j = i; j + 1 < count_; ++j) {
                const uint8_t from = static_cast<uint8_t>((head_ + j + 1) % kCapacity);
                const uint8_t to   = static_cast<uint8_t>((head_ + j) % kCapacity);
                ring_[to]          = ring_[from];
            }
            ring_[(head_ + count_ - 1) % kCapacity] = t;
            return;
        }
    }

    // Every slot holds an Up. Not reachable with this state machine -- an Up
    // can only follow a Down -- but if it ever were, dropping the NEW event is
    // the safe end: the queue already says "released".
}

bool TouchTracker::pop(TouchTransition& out)
{
    if (count_ == 0) {
        return false;
    }
    out   = ring_[head_];
    head_ = static_cast<uint8_t>((head_ + 1) % kCapacity);
    --count_;
    return true;
}

void TouchTracker::emit(TransitionKind kind, int16_t x, int16_t y, uint32_t nowMs,
                        bool synthetic)
{
    TouchTransition t{};
    t.kind        = kind;
    t.x           = x;
    t.y           = y;
    t.timestampMs = nowMs;
    t.synthetic   = synthetic;
    // sequence is assigned by push(), which is the only place that knows
    // whether this transition became a new queue entry or collapsed into one.

    switch (kind) {
    case TransitionKind::Down: ++downCount_; break;
    case TransitionKind::Move: ++moveCount_; break;
    case TransitionKind::Up:
        ++upCount_;
        if (synthetic) ++syntheticUpCount_;
        break;
    }

    push(t);

    // The snapshot always reflects the newest transition, whether or not the
    // UI has consumed it. A status screen reading this must never disturb the
    // queue the UI is draining.
    snapshot_.pressed     = (kind != TransitionKind::Up);
    snapshot_.x           = x;
    snapshot_.y           = y;
    snapshot_.timestampMs = nowMs;
    snapshot_.sequence    = t.sequence;
}

void TouchTracker::beginRelease(uint32_t nowMs, bool synthetic)
{
    if (!pressed_) {
        return;
    }
    pressed_ = false;
    // Released AT THE LAST KNOWN POSITION. LVGL decides what was clicked from
    // where the release happened, so handing it (0,0) would land every release
    // in the top-left corner.
    emit(TransitionKind::Up, lastX_, lastY_, nowMs, synthetic);
}

// ---------------------------------------------------------------- onFrame

void TouchTracker::onFrame(const ParsedFrame& frame, const TouchPoint& point, uint32_t nowMs)
{
    snapshot_.lastGesture = frame.gestureCode;

    // A frame that arrived intact but decoded to nonsense changes nothing.
    // Crucially it does NOT refresh lastContactMs_, so a run of corrupt frames
    // during a contact still ends in a synthesised release rather than a
    // widget stuck down.
    switch (frame.status) {
    case FrameStatus::AllOnes:
    case FrameStatus::BadFingerCount:
    case FrameStatus::ReservedEvent:
    case FrameStatus::OutOfRange:
        return;

    case FrameStatus::NoFinger:
        beginRelease(nowMs, false);
        return;

    case FrameStatus::Valid:
        break;
    }

    // Valid, but the chip may still be telling us the finger just left. See
    // TrackerConfig::trustEventUp for why Up wins over the finger count.
    if (frame.event == EventFlag::Up) {
        ++eventUpWithFingerCount_;
        if (config_.trustEventUp) {
            beginRelease(nowMs, false);
            return;
        }
    }

    if (!point.valid) {
        // Transform rejected it. Same treatment as a bad frame: no state
        // change, no refreshed contact time.
        return;
    }

    if (frame.event == EventFlag::Down && pressed_) {
        // A new press while we still believe the old one is held. The previous
        // Up was lost -- close it out at the OLD position before opening the
        // new one, or the UI sees one impossibly long drag across the screen.
        ++missedUpCount_;
        beginRelease(nowMs, true);
    }

    lastX_         = point.x;
    lastY_         = point.y;
    lastContactMs_ = nowMs;

    if (!pressed_) {
        pressed_ = true;
        emit(TransitionKind::Down, point.x, point.y, nowMs, false);
    } else {
        emit(TransitionKind::Move, point.x, point.y, nowMs, false);
    }
}

// ------------------------------------------------------------------- tick

void TouchTracker::tick(uint32_t nowMs)
{
    if (!pressed_) {
        return;
    }
    // Unsigned subtraction, so this stays correct across the 49-day wrap of a
    // millisecond counter.
    if ((nowMs - lastContactMs_) >= config_.releaseTimeoutMs) {
        beginRelease(nowMs, true);
    }
}

void TouchTracker::cancel(uint32_t nowMs)
{
    beginRelease(nowMs, true);
}

void TouchTracker::resetCounters()
{
    downCount_              = 0;
    upCount_                = 0;
    moveCount_              = 0;
    syntheticUpCount_       = 0;
    coalescedMoveCount_     = 0;
    overflowCount_          = 0;
    eventUpWithFingerCount_ = 0;
    missedUpCount_          = 0;
}

}  // namespace touch
