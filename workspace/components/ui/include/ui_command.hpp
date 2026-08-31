// What another task is allowed to say to the UI, and what it gets told back.
//
// Pure: <cstdint> only, so the lane policy is a host test rather than something
// discovered on a wrist.
//
// STATE GOES IN A TOPIC, EVENTS GO IN THIS QUEUE
//
// The line matters more than anything else in this file:
//
//   Topic  "what is true now"    battery 47%, wrist raised, 14:32
//          coalescing is CORRECT, a missed update is HARMLESS
//
//   Queue  "what just happened"  wake, alarm fired, user pressed
//          coalescing LOSES data, order matters, a miss is a BUG
//
// The first draft of ui.md put display state in the queue too and carried a
// flat UiStateSnapshot; both were wrong for a many-app watch. State moved to
// core::Topic (app-architecture.md section 4) and this queue kept only the half
// that genuinely cannot be coalesced.
#pragma once

#include <cstdint>

namespace ui {

enum class UiCommandType : uint8_t {
    // ---- control: order matters, never coalesced ----
    WAKE,
    SLEEP,
    DIM,
    SET_BRIGHTNESS,
    INVALIDATE_SCREEN,
    SHOW_APP,
    GO_BACK,
    GO_HOME,

    // ---- critical: must never be crowded out by anything above ----
    LOW_BATTERY,
    ALARM_FIRED,
    PREPARE_SHUTDOWN,
};

// Which lane a command travels in. Derived from the type rather than chosen by
// the sender: a caller that could pick its own priority would pick the high one.
enum class Lane : uint8_t {
    Control,
    Critical,
};

constexpr Lane laneOf(UiCommandType t)
{
    switch (t) {
    case UiCommandType::LOW_BATTERY:
    case UiCommandType::ALARM_FIRED:
    case UiCommandType::PREPARE_SHUTDOWN:
        return Lane::Critical;
    default:
        return Lane::Control;
    }
}

// Eight bytes, copyable, NO POINTERS -- and that is a rule, not an accident.
//
// A pointer here would be a lifetime nobody can verify at this boundary: the
// IMU task posts a pointer to its own stack frame, the UI task gets to it 30 ms
// later because it was mid-flush, and the frame is long gone. No compiler
// warning, no reproduction on a desk, and it only shows up when the machine is
// busy. Payloads that outgrow this get an immutable buffer owned by ui, or an
// index into a table owned by ui -- never the sender's memory.
struct UiCommand {
    UiCommandType type{};
    uint8_t       arg{};      // brightness 0..100, or an app index
    uint16_t      arg16{};
    uint32_t      sequence{};  // assigned by post(); lets a stale reply be dropped
};

// What post() actually did. Not void, and not a bare esp_err_t: a caller must
// not infer "delivered" from "did not crash", and dropping telemetry is a very
// different event from dropping a WAKE.
enum class PostResult : uint8_t {
    Accepted,   // queued
    Rejected,   // the runtime is not accepting commands (stopping/stopped)
    QueueFull,  // no room -- THE CALLER MUST HANDLE THIS
};

constexpr const char* toString(PostResult r)
{
    switch (r) {
    case PostResult::Accepted:  return "Accepted";
    case PostResult::Rejected:  return "Rejected";
    case PostResult::QueueFull: return "QueueFull";
    }
    return "?";
}

constexpr const char* toString(UiCommandType t)
{
    switch (t) {
    case UiCommandType::WAKE:              return "WAKE";
    case UiCommandType::SLEEP:             return "SLEEP";
    case UiCommandType::DIM:               return "DIM";
    case UiCommandType::SET_BRIGHTNESS:    return "SET_BRIGHTNESS";
    case UiCommandType::INVALIDATE_SCREEN: return "INVALIDATE_SCREEN";
    case UiCommandType::SHOW_APP:          return "SHOW_APP";
    case UiCommandType::GO_BACK:           return "GO_BACK";
    case UiCommandType::GO_HOME:           return "GO_HOME";
    case UiCommandType::LOW_BATTERY:       return "LOW_BATTERY";
    case UiCommandType::ALARM_FIRED:       return "ALARM_FIRED";
    case UiCommandType::PREPARE_SHUTDOWN:  return "PREPARE_SHUTDOWN";
    }
    return "?";
}

// Runtime lifecycle. Kept separate from display power state and from app
// lifecycle -- three orthogonal machines, so combinations like "runtime failed
// but the screen is still taking updates" cannot be expressed.
// doc-design/ui.md section 10.
enum class RuntimeState : uint8_t {
    Uninitialized,
    Starting,
    Running,
    Stopping,
    Stopped,
    Failed,
};

constexpr bool acceptsCommands(RuntimeState s)
{
    return s == RuntimeState::Starting || s == RuntimeState::Running;
}

constexpr const char* toString(RuntimeState s)
{
    switch (s) {
    case RuntimeState::Uninitialized: return "Uninitialized";
    case RuntimeState::Starting:      return "Starting";
    case RuntimeState::Running:       return "Running";
    case RuntimeState::Stopping:      return "Stopping";
    case RuntimeState::Stopped:       return "Stopped";
    case RuntimeState::Failed:        return "Failed";
    }
    return "?";
}

}  // namespace ui
