// The contract every app screen obeys. A contract, not a framework.
//
// WHY THIS EXISTS BEFORE THE FRAMEWORK DOES
//
// For a product whose direction is "many UI apps", the app contract IS the
// architecture. Write three apps ad hoc and you get three shapes, and the
// framework then gets retrofitted onto them -- which means rewriting all three
// AND ending up with a framework shaped by accidents of the first three.
//
// See doc-design/app-architecture.md sections 6 and 12.
#pragma once

#include "lvgl.h"

#include <cstdint>

namespace ui {

// ---------------------------------------------------------------------------
// THE TEN RULES
//
//  1. LAZY OBJECTS. An app's LVGL tree is built when it is first navigated to
//     and destroyed when it is evicted. 512 KB of SRAM with no PSRAM does not
//     allow "construct every app at registration".
//
//  2. FOUR PHASES, NOT TWO. onCreate / onEnter / onExit / onDestroy. The
//     enter/exit pair is what makes a BACKGROUND app cheap: its objects
//     survive, its work does not.
//
//  3. THE FRAMEWORK OWNS THE ROOT. It creates `root`, passes it to onCreate(),
//     and deletes it. An app must never lv_obj_delete(root). Deleting a parent
//     takes the whole subtree with it, so onDestroy() MUST null every cached
//     child pointer.
//
//  4. A BACKGROUND APP RUNS NOTHING. Timers and animations stop at onExit().
//     Otherwise fifteen apps tick in the background and the watch never sleeps.
//
//     Work that must continue while the app is off screen goes in the app's
//     DAEMON half instead -- see IAppDaemon below. That half never touches
//     LVGL, which is what keeps this rule and background work compatible.
//
//  5. DATA COMES FROM TOPICS. Subscribe in onEnter(), release in onExit(). An
//     app never holds a pointer into a service's internals, and never a pointer
//     to a driver.
//
//  6. EFFECTS GO THROUGH SERVICES. Never a driver call. An app that reaches
//     BuzzerManager directly is an app that fights the next app for the buzzer
//     with no referee.
//
//  7. THE FRAMEWORK DECIDES NAVIGATION. An app REQUESTS; it never calls
//     lv_screen_load(). Requests carry a sequence so a stale one loses.
//
//  8. MODALS LIVE IN A GLOBAL LAYER, never parented to an app's root.
//
//  9. OUT OF MEMORY IS A DEFINED PATH. Checked BEFORE building: refuse the
//     navigation, stay put, say so. Never a crash, never a half-built screen.
//
// 10. EVERY APP HAS A RAM BUDGET, measured around onCreate() and logged.
// ---------------------------------------------------------------------------

// Why an app is being torn down. Lets an app tell "the user left" from "the
// system needs your memory" -- only the second justifies throwing away
// expensive cached work.
enum class ExitReason : uint8_t {
    Navigated,      // the user went somewhere else
    Evicted,        // memory pressure; onDestroy() follows immediately
    Suspending,     // screen going off; the app stays constructed
    ShuttingDown,
};

// ---------------------------------------------------------------------------
// The UI half. Runs in the UI task, only while in the foreground.
// ---------------------------------------------------------------------------
class IApp {
public:
    virtual ~IApp() = default;

    // Short, stable, unique. Used in logs and by the registry.
    virtual const char* id() const = 0;

    // Build the object tree under `root`. Runs in the UI task.
    //
    // MUST NOT block: no I2C, no filesystem, no network. A slow onCreate() is a
    // dropped frame at best and a watchdog at worst; expensive data is prepared
    // by a service and read from a topic.
    //
    // NO CONTEXT PARAMETER, deliberately. Dependencies arrive once, through the
    // app's constructor -- an earlier draft passed an AppContext to every
    // lifecycle method and every app ignored it, which is how you can tell a
    // parameter is decoration rather than design.
    virtual void onCreate(lv_obj_t* root) = 0;

    // Becoming visible: subscribe to topics, start timers and animations.
    virtual void onEnter() = 0;

    // Leaving the foreground. MUST stop every timer and animation it started --
    // rule 4. After this returns the app is expected to consume no CPU.
    virtual void onExit(ExitReason reason) = 0;

    // The tree is about to be deleted by the framework. Forget every cached
    // child pointer -- rule 3. Do NOT delete root.
    virtual void onDestroy() = 0;

    // Once per frame while in the foreground, after input has been sampled.
    virtual void onTick(uint32_t nowMs) = 0;
};

// What almost every app should inherit from.
//
// IApp states the contract in full so it can be read in one place; AppBase
// makes the parts an app does not care about free. Only id() and onCreate()
// stay mandatory -- an app with neither an identity nor a screen is not an app.
class AppBase : public IApp {
public:
    void onEnter() override {}
    void onExit(ExitReason) override {}
    void onDestroy() override {}
    void onTick(uint32_t) override {}
};

// ---------------------------------------------------------------------------
// The background half lives elsewhere: components/app_daemon/daemon.hpp
// ---------------------------------------------------------------------------
//
// IAppDaemon used to be declared right here, and that placement encoded a
// design that has since been replaced. The old daemon had one method -- attach
// yourself, at boot, to something that already has a task -- which in practice
// meant the IMU fan-out, which meant every app's background code ran in the IMU
// task: priority 10, ABOVE the UI, on a driver's 4096-byte stack.
//
// A daemon now gets a task of its own, with its own stack and a priority the
// framework clamps below the UI's. That runtime has no business depending on
// LVGL, so it does not live in this header, which does.
//
// It is still declared by the app, still ships in the app's own component, and
// is still wired up in one line at the composition root. See daemon.hpp and
// doc-design/app-architecture.md section 5.

}  // namespace ui
