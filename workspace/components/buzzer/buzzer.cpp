#include "buzzer.hpp"

namespace buzzer {

Buzzer::Buzzer(const BuzzerConfig& config) : config_(config) {}

void Buzzer::reset()
{
    pattern_        = Pattern{};
    state_          = BuzzerState::IDLE;
    outputPending_  = false;
    infinite_       = false;
    noteIndex_      = 0;
    repeatLeft_     = 0;
    noteStartMs_    = 0;
    noteDurationMs_ = 0;
}

uint8_t Buzzer::clampVolume(uint8_t volume) const
{
    const uint8_t ceiling = config_.maxVolume > 100 ? 100 : config_.maxVolume;
    return volume > ceiling ? ceiling : volume;
}

// freqHz == 0 is a rest whatever the device is. Beyond that an ACTIVE buzzer
// oscillates on its own, so any non-zero value simply means "drive the pin";
// only a PASSIVE one has a frequency range that can be out of bounds.
bool Buzzer::isAudible(uint16_t freqHz) const
{
    if (freqHz == 0) {
        return false;
    }
    if (config_.type == BuzzerType::ACTIVE) {
        return true;
    }
    return freqHz >= config_.minFreqHz && freqHz <= config_.maxFreqHz;
}

void Buzzer::loadNote(uint32_t nowMs)
{
    // Resync on the real clock instead of chaining deadlines. Chaining would be
    // drift free, but a task that stalls for a second would then machine-gun a
    // whole pattern to catch up. Beeps are short; a few ms of drift is inaudible,
    // a burst of notes is not.
    noteStartMs_ = nowMs;

    // A zero-length note would make nextDelayMs() return 0 forever and the driver
    // task would spin instead of sleeping. One millisecond costs nothing and
    // keeps the sequence moving.
    const uint16_t d = pattern_.notes[noteIndex_].durationMs;
    noteDurationMs_  = (d == 0) ? static_cast<uint16_t>(1) : d;
}

bool Buzzer::play(const Pattern& pattern, uint32_t nowMs)
{
    if (pattern.notes == nullptr || pattern.count == 0) {
        return false;
    }

    // Strictly lower loses; equal wins. An alarm cannot be cut short by a UI
    // click, but re-triggering the same alarm restarts it.
    if (state_ == BuzzerState::PLAYING && pattern.priority < pattern_.priority) {
        return false;
    }

    pattern_    = pattern;
    state_      = BuzzerState::PLAYING;
    infinite_   = (pattern.repeat == 0);
    noteIndex_  = 0;
    repeatLeft_ = pattern.repeat;
    loadNote(nowMs);

    // The first note has not reached the hardware yet; update() must emit it.
    outputPending_ = true;
    return true;
}

void Buzzer::stop()
{
    if (state_ == BuzzerState::IDLE) {
        return;  // already silent: do not queue a redundant hardware write
    }
    pattern_       = Pattern{};
    state_         = BuzzerState::IDLE;
    outputPending_ = true;  // exactly one OFF command still has to go out
}

// Move to the next note, or to the next repetition. Returns false when the
// pattern is finished for good.
bool Buzzer::advance(uint32_t nowMs)
{
    ++noteIndex_;
    if (noteIndex_ < pattern_.count) {
        loadNote(nowMs);
        return true;
    }

    // One full pass through the pattern just ended.
    noteIndex_ = 0;
    if (!infinite_) {
        if (repeatLeft_ > 0) {
            --repeatLeft_;
        }
        if (repeatLeft_ == 0) {
            return false;
        }
    }
    loadNote(nowMs);
    return true;
}

ToneOutput Buzzer::currentOutput() const
{
    ToneOutput out{};
    out.changed = true;  // this call exists precisely to force a hardware write

    if (state_ != BuzzerState::PLAYING) {
        return out;  // on = false: silence
    }

    const Note&   note = pattern_.notes[noteIndex_];
    const uint8_t vol  = clampVolume(note.volume);

    // A rest, a zero volume note or a frequency the device cannot produce all
    // stay silent, yet still burn their full durationMs. That is what makes the
    // gap between two beeps expressible as an ordinary Note.
    if (vol == 0 || !isAudible(note.freqHz)) {
        return out;
    }

    out.on     = true;
    out.freqHz = note.freqHz;
    out.volume = vol;
    return out;
}

ToneOutput Buzzer::update(uint32_t nowMs)
{
    // Subtraction only, never 'nowMs >= deadline': the difference stays correct
    // when nowMs wraps past UINT32_MAX (same rule as Button, section 9).
    if (state_ == BuzzerState::PLAYING &&
        (nowMs - noteStartMs_) >= noteDurationMs_) {
        if (!advance(nowMs)) {
            pattern_ = Pattern{};  // clears priority too: nothing outranks silence
            state_   = BuzzerState::IDLE;
        }
        outputPending_ = true;
    }

    if (!outputPending_) {
        return ToneOutput{};  // changed = false: leave the hardware alone
    }
    outputPending_ = false;
    return currentOutput();
}

uint32_t Buzzer::nextDelayMs(uint32_t nowMs) const
{
    // The pending check comes FIRST, and it is not optional. After stop() the
    // state is already IDLE while the OFF command has not been applied; sleeping
    // here would leave the piezo screaming until the next command arrives.
    if (outputPending_) {
        return 0;
    }
    if (state_ != BuzzerState::PLAYING) {
        return kSleepForever;
    }

    const uint32_t elapsed = nowMs - noteStartMs_;
    if (elapsed >= noteDurationMs_) {
        return 0;  // already overdue: update() must run now, not after a sleep
    }
    return static_cast<uint32_t>(noteDurationMs_) - elapsed;
}

}  // namespace buzzer
