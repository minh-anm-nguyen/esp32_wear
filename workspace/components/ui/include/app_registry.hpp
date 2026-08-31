// What apps exist. Filled in at composition time, read-only afterwards.
//
// Fixed capacity, no heap, registered before the UI task starts -- the same
// shape and the same reason as sensors::SampleFanout: a list that is written
// once during bring-up and only read afterwards needs no locking, and giving it
// none is what makes that guarantee checkable.
#pragma once

#include "app.hpp"
#include "daemon_host.hpp"

#include <cstdint>

namespace ui {

// Build these with DESIGNATED INITIALISERS:
//
//     registry.add({.id = "wrist", .title = "Co tay",
//                   .icon = LV_SYMBOL_EYE_OPEN, .ui = &wristApp})
//
// Not a style preference. id, title and icon are all const char*, so positional
// initialisation lets any two of them be swapped and still compile -- and the
// only symptom is a launcher tile with the wrong caption, which nobody reads as
// a code bug. Naming the fields makes that class of mistake impossible.
struct AppEntry {
    const char* id{nullptr};     // stable, for logs
    const char* title{nullptr};  // shown under the icon
    const char* icon{nullptr};   // an LV_SYMBOL_* string

    IApp* ui{nullptr};

    // What this app may cost in the LVGL pool, in bytes. 0 takes the host's
    // default. Set it when an app is legitimately heavier than the default and
    // you want the number reviewed here, in the registry, rather than
    // discovered as a warning nobody can attribute months later.
    uint32_t maxLvglBytes{0};

    // Optional. Gets a task of its own at boot and is never stopped, so it
    // keeps working while the app is off screen -- app.hpp rule 4.
    //
    // Fully qualified because the member is called `daemon` too, and a member
    // name hides the namespace it shadows for everything declared after it.
    ::background::IAppDaemon* daemon{nullptr};
};

// Non-template base so AppHost can hold one without becoming a template itself.
class AppRegistryBase {
public:
    virtual ~AppRegistryBase()          = default;
    virtual uint8_t         count() const = 0;
    virtual const AppEntry& at(uint8_t index) const = 0;

    int8_t indexOf(const char* id) const;
};

// Hands every daemon in the registry to the DaemonHost, which is what gives
// each of them a task.
//
// Lives here rather than in app_daemon because the dependency has to point this
// way: ui knows about both a registry and a daemon host, while the daemon
// runtime must stay unaware of apps, launchers and LVGL entirely.
//
// Call from the composition root BEFORE DaemonHost::start(). Returns how many
// were accepted, so the caller can log a number rather than hope.
uint8_t registerDaemons(AppRegistryBase& registry, ::background::DaemonHost& host);

template <uint8_t N>
class AppRegistry final : public AppRegistryBase {
    static_assert(N > 0, "a registry with no room has no launcher to show");

public:
    // [[nodiscard]] because the first version of main.cpp ignored this at all
    // three call sites -- while this very comment told the caller not to. An
    // app that fails to register produces no error at runtime; it simply never
    // appears, and a missing tile does not read as a code bug. Now it does not
    // compile.
    [[nodiscard]] bool add(const AppEntry& e)
    {
        if (count_ >= N || e.ui == nullptr || e.id == nullptr) {
            return false;
        }
        entries_[count_++] = e;
        return true;
    }

    uint8_t         count() const override { return count_; }
    const AppEntry& at(uint8_t index) const override { return entries_[index]; }
    uint8_t         capacity() const { return N; }

private:
    AppEntry entries_[N]{};
    uint8_t  count_{0};
};

}  // namespace ui
