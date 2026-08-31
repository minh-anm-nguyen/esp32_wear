// Runs apps: owns their root object, drives the four lifecycle phases, and is
// the only thing allowed to decide what is on screen.
//
// This is the machinery for the contract in app.hpp. Everything it does is
// stated there as a numbered rule; this header notes which rule each piece
// implements so the two cannot drift apart.
//
// RUNS ENTIRELY IN THE UI TASK. Every method here touches LVGL. Other tasks
// ask for navigation by posting a UiCommand -- app.hpp rule 7.
#pragma once

#include "app.hpp"
#include "app_registry.hpp"
#include "crash_log.hpp"
#include "nav_stack.hpp"
#include "ui_context.hpp"

#include "lvgl.h"

#include <cstdint>

namespace ui {

// How many apps deep the back stack goes. Three is generous for a watch: the
// launcher plus an app plus one sub-screen. Deeper than that and a user has no
// idea how many times Back has to be pressed.
inline constexpr uint8_t kNavDepth = 3;

struct AppHostConfig {
    // Rule 9: refuse BEFORE building, never halfway through.
    //
    // Against LVGL's 64 KB pool. The bring-up screen used about 1.5 KB, so this
    // is a wide margin -- it exists to keep a heavy screen from consuming the
    // last of the pool and leaving the launcher unable to rebuild itself, which
    // would strand the user in whatever app they were in.
    uint32_t minLvglFreeToOpen{12 * 1024};

    // RULE 10 WITH TEETH: what ONE app may cost in the 64 KB LVGL pool.
    //
    // 8 KB is generous against what the apps here actually use (the heaviest is
    // under 3 KB) and is not meant to be tight. It is meant to be a number a
    // developer trips over while they still have the code in their head,
    // instead of discovering the cost months later as "the launcher sometimes
    // refuses to open".
    //
    // Exceeding it is reported, never fatal: the app opens, and the log names
    // it and says by how much.
    uint32_t defaultAppQuotaBytes{8 * 1024};

    // The point at which one app is threatening every other app. An app that
    // crosses this is opened once -- it is already built by the time we know --
    // and then LOCKED OUT, so the next attempt is refused and the watch stays
    // usable. app.hpp rule 9.
    uint32_t hardAppCeilingBytes{20 * 1024};
};

// Per-app memory accounting. One record per registry entry, plus the launcher.
//
// WHY residueBytes IS THE INTERESTING ONE
//
// peakBytes says what an app costs while it is open, which is a budgeting
// question. residueBytes says what it FAILED TO GIVE BACK after it was torn
// down, which is a correctness question -- and it is the number that decides
// whether a watch survives a week of use or slowly stops being able to open
// anything.
//
// It is measurable here only because exactly one app is ever resident: the host
// destroys the outgoing app on every transition, so returning to the launcher
// must always leave the same amount of memory free. Any drift is a leak, and
// the app visited in between is the one that leaked it.
struct AppMemStats {
    uint32_t opens{0};
    uint32_t lastBytes{0};      // cost of the most recent onCreate + onEnter
    uint32_t peakBytes{0};
    uint32_t overBudget{0};     // times it exceeded its quota
    int32_t  residueBytes{0};   // never handed back; positive is bad
    bool     lockedOut{false};  // crossed the hard ceiling
};

// How many registry entries get their own accounting record.
inline constexpr uint8_t kMaxTrackedApps = 8;

class AppHost {
public:
    AppHost()                          = default;
    AppHost(const AppHost&)            = delete;
    AppHost& operator=(const AppHost&) = delete;

    // `launcher` is an ordinary IApp that the host owns specially: it is what
    // the stack sits on, so it is never pushed and never destroyed by Back.
    void attach(AppRegistryBase* registry, IApp* launcher,
                const AppHostConfig& cfg = {});

    // Must be the first thing shown. Returns false when even the launcher
    // cannot be built, which is a dead UI and worth logging loudly.
    bool start();

    // ---- navigation. UI task only. ----
    bool showApp(uint8_t index);
    bool goBack();
    bool goHome();

    // Once a frame, after commands are drained. Forwards to the foreground app
    // only -- a background app runs NOTHING (rule 4).
    void tick(uint32_t nowMs);

    int8_t      currentIndex() const { return nav_.current(); }
    const char* currentId() const;
    uint32_t    refusedForMemory() const { return refusedForMemory_; }
    uint32_t    transitions() const { return transitions_; }
    uint32_t    peakAppLvglBytes() const { return peakAppBytes_; }

    // Total bytes the LVGL pool has never got back since the first time the
    // launcher was shown. Zero on a healthy build; a number that only grows is
    // the one bug that cannot be found by looking at a single screenshot.
    int32_t leakedBytes() const { return leaked_; }

    const AppMemStats& statsFor(int8_t index) const;

    void logDiagnostics(const char* where) const;

private:
    // Builds a screen for `app`, swaps to it, then tears the old one down.
    //
    // ORDER MATTERS AND IS NOT OBVIOUS. The new screen is built BEFORE the old
    // app is told it is leaving, so that a refusal or a failure leaves the user
    // exactly where they were rather than on a blank screen with two
    // half-dismantled apps. Rule 9.
    bool activate(IApp* app, ExitReason reasonForOutgoing);

    IApp*       appAt(int8_t index) const;
    const char* idAt(int8_t index) const;

    AppRegistryBase*  registry_{nullptr};
    IApp*             launcher_{nullptr};
    AppHostConfig     cfg_{};

    NavStack<kNavDepth> nav_{};

    IApp*     currentApp_{nullptr};
    lv_obj_t* currentScreen_{nullptr};

    AppMemStats* mutableStatsFor(int8_t index);
    uint32_t     quotaFor(int8_t index) const;

    uint32_t transitions_{0};
    uint32_t refusedForMemory_{0};
    uint32_t peakAppBytes_{0};

    // Slot kMaxTrackedApps is the launcher's; apps take 0..kMaxTrackedApps-1.
    AppMemStats stats_[kMaxTrackedApps + 1]{};

    // The clean measuring point: free memory with ONLY the launcher resident.
    // Recorded the first time the launcher is shown and compared on every
    // return to it.
    uint32_t baselineFree_{0};
    uint32_t lastLauncherFree_{0};
    bool     haveBaseline_{false};
    int32_t  leaked_{0};

    // Which app is resident right now, as an index. nav_ knows where the USER
    // is, which is the same thing except during a refused transition.
    int8_t activeIndex_{kLauncherIndex};
};

}  // namespace ui
