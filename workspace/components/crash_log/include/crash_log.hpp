// Which app was on screen when the board died.
//
// WHY THIS EXISTS
//
// There is no MMU on the ESP32-S3 and no memory protection between FreeRTOS
// tasks, so two whole classes of app bug cannot be contained: a stack overflow
// and a wild pointer both end the same way, in the panic handler, with the
// board rebooting. daemon.hpp states that plainly and says what the design CAN
// do about it -- make the failure name its author.
//
// For a DAEMON the panic report already does: its task is named after the
// daemon. For a UI app it does not, and cannot: every app runs in the task
// called "ui", because CONFIG_LV_OS_NONE makes one UI task mandatory. So the
// panic report for fifteen different apps looks identical.
//
// This closes that. One string in RTC memory, which survives a reset, saying
// what was on screen. On the next boot the log opens with the answer instead of
// with a mystery.
//
// AND THEN IT ACTS ON IT
//
// An app that takes the board down three times in a row is QUARANTINED: the
// host refuses to open it and says why. That is the only fault containment
// available for a class of bug the hardware will not contain -- the watch stays
// usable, every other app still runs, and the broken one stops being a boot
// loop. Running to a clean exit clears the record, so a one-off does not
// condemn an app forever.
//
// SURVIVES: software reset, panic, watchdog, deep sleep.
// DOES NOT SURVIVE: power loss. The magic word is what tells the two apart, and
// a cold boot legitimately starts with a clean sheet.
#pragma once

#include <cstdint>

namespace forensics {

// Consecutive crashes with the same app on screen before it is locked out.
//
// Three, not two. Two consecutive panics can share an external cause -- a
// brownout while the backlight ramps, a bad flash. Three with the same app
// named each time is the app.
inline constexpr uint8_t kQuarantineStrikes = 3;

// Boots after which a quarantine expires and the app is allowed to try again.
//
// Without this the lock-out is permanent: a quarantined app cannot be opened,
// so it can never run clean, so it can never clear its own record -- and RTC
// memory survives every reset. The point of the lock-out is to break a boot
// loop, not to delete a feature, and the evidence behind it is circumstantial
// enough that a life sentence is the wrong call.
inline constexpr uint32_t kQuarantineBoots = 5;

// Reads what the previous boot left behind, prints it, and arms the record for
// this run. Call ONCE, as early in app_main as logging allows.
void reportBoot();

// The app the user is looking at right now. Cheap: a bounded string copy into
// RTC memory, no lock. Called by AppHost on every transition.
void setCurrentApp(const char* id);

// How many consecutive crashes are attributed to this app.
uint8_t strikes(const char* id);

// True when this app has earned its way out of being opened.
bool quarantined(const char* id);

// This app ran and left without taking the board down. Clears its record.
void clearStrikes(const char* id);

// For diagnostics: what crashed last, and how many times this board has booted.
const char* culprit();
uint32_t    bootCount();

}  // namespace forensics
