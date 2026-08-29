// Pure logic layer: a sequencer that walks a pattern of notes over time.
//
// This header includes nothing but <cstddef> and <cstdint>. No LEDC, no
// FreeRTOS, and no #ifdef ESP_PLATFORM shim -- it used to fake a gpio_num_t for
// host builds purely because BuzzerConfig carried a `pin` the sequencer never
// read. The wiring now lives in buzzer_manager.hpp and the shim is gone.
//
// Mirror image of button.hpp: Button turns pin levels into events, Buzzer turns
// elapsed time into output commands. Same three layers, opposite direction.
// See doc-design/button-esp-idf-design.v2.md for the conventions reused here.
#pragma once

#include <cstddef>
#include <cstdint>

namespace buzzer {

// ------------------------------------------------------------------ data types

enum class BuzzerType : uint8_t {
    ACTIVE,   // self-oscillating buzzer or a vibration motor: only needs a level
    PASSIVE,  // piezo disc: needs a PWM square wave at the wanted frequency
};

enum class BuzzerState : uint8_t {
    IDLE,     // no pattern loaded, the device is silent
    PLAYING,  // walking through the notes of a pattern
};

struct Note {
    uint16_t freqHz{};
    uint16_t durationMs{};
    uint8_t  volume{100};  // 0..100, the driver layer turns it into a duty cycle
};

// freqHz == 0 is a REST for every device type: stay silent for durationMs.
// Encoding a gap as an ordinary Note instead of a separate field keeps every
// element of a pattern on one code path.
//
// For BuzzerType::ACTIVE the frequency carries no meaning at all, so use kOn to
// say "drive the pin" while still honouring the 0-means-rest rule.
inline constexpr uint16_t kOn = 1;

// Non-owning, like std::span. 'notes' normally points at a constexpr array in
// flash.
//
// ITS LIFETIME MUST OUTLIVE THE PLAYBACK. A Pattern built from a local array and
// handed to BuzzerManager::play() is a dangling pointer by the time the buzzer
// task reads it, because the queue copies the Pattern but not the notes. Use
// static or constexpr storage.
struct Pattern {
    const Note* notes{nullptr};
    uint8_t     count{0};
    uint8_t     repeat{1};    // 0 = loop until stop()
    uint8_t     priority{0};  // a higher value may cut into a lower one
};

// Deduces the element count, so a Pattern can never disagree with its array.
template <std::size_t N>
constexpr Pattern makePattern(const Note (&notes)[N],
                              uint8_t repeat   = 1,
                              uint8_t priority = 0)
{
    static_assert(N > 0 && N <= 255, "a Pattern holds between 1 and 255 notes");
    return Pattern{notes, static_cast<uint8_t>(N), repeat, priority};
}

// What the DEVICE can do -- the four facts the sequencer needs in order to
// decide whether a note is playable and how loud it may be.
//
// Nothing about how it is wired: that is BuzzerWiring, in buzzer_manager.hpp.
// Splitting the two is what let this header drop its gpio_num_t shim.
struct BuzzerSpec {
    BuzzerType type{BuzzerType::PASSIVE};
    uint8_t    maxVolume{100};     // hardware ceiling, clamps every Note::volume
    uint16_t   minFreqHz{100};     // PASSIVE only: outside the range -> silent
    uint16_t   maxFreqHz{10000};
};

// What the driver layer must apply after each update().
//
// changed == false means "hold, touch nothing". That flag is not a micro
// optimisation: every ledc_set_freq() reloads the timer divider, so re-applying
// an unchanged tone would put an audible click into the middle of a note.
struct ToneOutput {
    bool     changed{false};
    bool     on{false};
    uint16_t freqHz{0};
    uint8_t  volume{0};
};

// nextDelayMs() returns this when there is nothing scheduled at all.
inline constexpr uint32_t kSleepForever = UINT32_MAX;

// ---------------------------------------------------------------------- Buzzer

class Buzzer {
public:
    explicit Buzzer(const BuzzerSpec& spec);

    // Returns false when refused: an empty pattern, or one whose priority is
    // strictly below the pattern currently playing. Equal priority replaces on
    // purpose -- pressing the same button twice should restart its beep rather
    // than be swallowed.
    bool play(const Pattern& pattern, uint32_t nowMs);

    // Always succeeds. The next update() emits exactly one OFF command.
    void stop();

    // Call when nextDelayMs() has elapsed, and again right after play()/stop().
    // Returns changed == true only at a note boundary.
    ToneOutput update(uint32_t nowMs);

    // How long until there is something to do; kSleepForever means "never".
    // This is what replaces ButtonManager's fixed poll interval: a buzzer knows
    // in advance every instant at which it must act, a button never does.
    uint32_t nextDelayMs(uint32_t nowMs) const;

    // The command describing the CURRENT note, with changed forced to true. The
    // driver needs it to re-apply state out of band -- muting has to take effect
    // immediately, not when the two second alarm note happens to end.
    ToneOutput currentOutput() const;

    // The '&& !outputPending_' term is mandatory, for the same reason as the
    // 'debounceCounter_ == 0' term in Button::isIdle(): after stop() the state
    // is already IDLE while the OFF command has not reached the hardware yet.
    bool isIdle() const
    {
        return state_ == BuzzerState::IDLE && !outputPending_;
    }

    BuzzerState getState() const { return state_; }

    const BuzzerSpec& spec() const { return spec_; }

    // Index of the note being played. Meaningless while IDLE.
    uint8_t noteIndex() const { return noteIndex_; }

    void reset();

private:
    bool    isAudible(uint16_t freqHz) const;
    uint8_t clampVolume(uint8_t volume) const;
    void    loadNote(uint32_t nowMs);
    bool    advance(uint32_t nowMs);

    BuzzerSpec spec_;
    Pattern    pattern_{};

    BuzzerState state_{BuzzerState::IDLE};
    bool        outputPending_{false};
    bool        infinite_{false};
    uint8_t     noteIndex_{0};
    uint8_t     repeatLeft_{0};
    uint32_t    noteStartMs_{0};
    uint16_t    noteDurationMs_{0};
};

// ------------------------------------------------------------- stock patterns

// constexpr, so these live in flash and cost no RAM. Priorities are spaced out
// to leave room for application specific patterns in between.
namespace patterns {

// PASSIVE (piezo). 2700 Hz is the resonant peak of the common 12 mm disc, which
// is why the UI beeps sit there: same loudness for far less current.
inline constexpr Note kClickNotes[] = {{2700, 30, 60}};
inline constexpr Note kOkNotes[]    = {{2000, 80, 80}, {0, 40, 0}, {2700, 120, 80}};
inline constexpr Note kErrorNotes[] = {{400, 150, 90}, {0, 80, 0}, {400, 150, 90}};
inline constexpr Note kAlarmNotes[] = {{2700, 200, 100}, {0, 150, 0},
                                       {2700, 200, 100}, {0, 700, 0}};

inline constexpr Pattern kClick = makePattern(kClickNotes, 1, 10);
inline constexpr Pattern kOk    = makePattern(kOkNotes, 1, 20);
inline constexpr Pattern kError = makePattern(kErrorNotes, 1, 50);
inline constexpr Pattern kAlarm = makePattern(kAlarmNotes, 0, 200);  // until stop()

// ACTIVE (vibration motor). Frequency is meaningless here, kOn just means
// "drive the pin"; the same class and the same Pattern type drive both devices.
inline constexpr Note kBuzzShortNotes[] = {{kOn, 120, 100}};
inline constexpr Note kBuzzTwiceNotes[] = {{kOn, 100, 100}, {0, 120, 0},
                                           {kOn, 100, 100}};

inline constexpr Pattern kBuzzShort = makePattern(kBuzzShortNotes, 1, 10);
inline constexpr Pattern kBuzzTwice = makePattern(kBuzzTwiceNotes, 1, 20);

}  // namespace patterns

}  // namespace buzzer
