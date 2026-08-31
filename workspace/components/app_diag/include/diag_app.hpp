// Health of the UI runtime, on the watch instead of in a serial log.
//
// Reads a snapshot through a provider callback rather than holding a pointer to
// UiManager: an app must not be able to reach into the runtime and change it.
// Same shape as every other app -- read something, render what you read.
#pragma once

#include "app.hpp"

#include <cstdint>

namespace apps {

// Filled in by whoever owns the runtime. Small, POD, copied.
struct DiagSnapshot {
    uint32_t frames{};
    uint32_t flushPerFrame100{};   // flushes per frame x100
    uint32_t stackFreeBytes{};
    uint32_t lvglFreeBytes{};
    uint32_t lvglLargestBytes{};
    uint32_t sramFreeBytes{};
    uint32_t sramLargestBytes{};
    bool     flushBalanced{true};
};

using DiagFn = void (*)(void* ctx, DiagSnapshot& out);

class DiagApp final : public ui::AppBase {
public:
    DiagApp(DiagFn fn, void* ctx) : fn_(fn), ctx_(ctx) {}

    const char* id() const override { return "diag"; }

    void onCreate(lv_obj_t* root) override;
    void onEnter() override;
    void onDestroy() override;
    void onTick(uint32_t nowMs) override;

private:
    DiagFn fn_{nullptr};
    void*  ctx_{nullptr};

    lv_obj_t* body_{nullptr};
    uint32_t  lastRenderMs_{0};
};

}  // namespace apps
