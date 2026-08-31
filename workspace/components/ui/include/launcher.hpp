// The home screen: a grid of app icons.
//
// The launcher is itself an IApp, deliberately. One lifecycle mechanism for
// everything on screen means the host has no special case, and the launcher
// gets the same memory accounting and the same teardown rules as any app.
//
// What makes it special is only its POSITION: it is the floor of the navigation
// stack (nav_stack.hpp), so Back never destroys it and it is never pushed.
//
// Icons are LV_SYMBOL_* strings from the built-in Montserrat font. No asset
// pipeline, no image decoder, no extra flash beyond the font itself -- and
// nothing to build before the first app can be seen. Real artwork is a later
// decision (app-architecture.md section 13, item 10).
#pragma once

#include "app.hpp"
#include "app_registry.hpp"

#include "lvgl.h"

namespace ui {

class Launcher final : public AppBase {
public:
    // Called when the user picks a tile. The launcher REPORTS; the host DECIDES
    // -- app.hpp rule 7.
    using PickFn = void (*)(void* ctx, uint8_t appIndex);

    void attach(AppRegistryBase* registry, PickFn onPick, void* ctx);

    const char* id() const override { return "launcher"; }

    void onCreate(lv_obj_t* root) override;
    void onDestroy() override;

private:
    static void onTileClicked(lv_event_t* e);

    AppRegistryBase* registry_{nullptr};
    PickFn           onPick_{nullptr};
    void*            pickCtx_{nullptr};
};

}  // namespace ui
