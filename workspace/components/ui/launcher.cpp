#include "launcher.hpp"

#include "esp_log.h"

namespace ui {
namespace {
constexpr const char* TAG = "launcher";

// 240 wide, two columns: 6 px page padding either side, an 8 px seam between
// the tiles, and 104 left for each tile.
constexpr int32_t kTileW   = 104;
constexpr int32_t kTileH   = 92;
constexpr int32_t kTileGap = 8;
}  // namespace

void Launcher::attach(AppRegistryBase* registry, PickFn onPick, void* ctx)
{
    registry_ = registry;
    onPick_   = onPick;
    pickCtx_  = ctx;
}

void Launcher::onTileClicked(lv_event_t* e)
{
    auto* self = static_cast<Launcher*>(lv_event_get_user_data(e));

    // The index travels in the tile's own user data rather than in a lambda
    // capture: LVGL callbacks are C function pointers, and a capturing lambda
    // cannot become one. Storing it on the object also means the value cannot
    // outlive the object it describes.
    lv_obj_t*     tile  = static_cast<lv_obj_t*>(lv_event_get_target(e));
    const uint32_t index = reinterpret_cast<uintptr_t>(lv_obj_get_user_data(tile));

    if (self->onPick_ != nullptr) {
        // Does NOT navigate here. The launcher only reports a user intent; the
        // host decides -- app.hpp rule 7. Navigating from inside an event
        // callback would also mean deleting this very object tree while LVGL is
        // still dispatching an event on it.
        self->onPick_(self->pickCtx_, static_cast<uint8_t>(index));
    }
}

void Launcher::onCreate(lv_obj_t* root)
{
    lv_obj_set_style_bg_color(root, lv_color_hex(0x101014), LV_PART_MAIN);
    lv_obj_set_style_pad_all(root, 6, LV_PART_MAIN);
    lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);

    // Wrapping row flex: two tiles per row on a 240 px panel, and it reflows on
    // its own as apps are added rather than needing a hand-computed grid.
    lv_obj_set_flex_flow(root, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(root, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scrollbar_mode(root, LV_SCROLLBAR_MODE_OFF);

    // Flex gaps default to zero, which put the tiles edge to edge: a finger
    // landing on the boundary hits whichever side won by a pixel. Worth the
    // eight pixels -- while the touch transform was still the identity, a tile
    // was unreachable because its mirror image missed the next tile by exactly
    // one pixel and fell into the seam.
    lv_obj_set_style_pad_column(root, kTileGap, LV_PART_MAIN);
    lv_obj_set_style_pad_row(root, kTileGap, LV_PART_MAIN);

    if (registry_ == nullptr) {
        return;
    }

    for (uint8_t i = 0; i < registry_->count(); ++i) {
        const AppEntry& entry = registry_->at(i);

        lv_obj_t* tile = lv_button_create(root);
        lv_obj_set_size(tile, kTileW, kTileH);
        lv_obj_set_style_bg_color(tile, lv_color_hex(0x1E2430), LV_PART_MAIN);
        lv_obj_set_style_radius(tile, 12, LV_PART_MAIN);
        lv_obj_set_style_pad_all(tile, 4, LV_PART_MAIN);

        // The index, carried by the object it belongs to.
        lv_obj_set_user_data(tile, reinterpret_cast<void*>(static_cast<uintptr_t>(i)));
        lv_obj_add_event_cb(tile, &Launcher::onTileClicked, LV_EVENT_CLICKED, this);

        lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);

        lv_obj_t* icon = lv_label_create(tile);
        lv_label_set_text(icon, entry.icon != nullptr ? entry.icon : LV_SYMBOL_DUMMY);
        lv_obj_set_style_text_font(icon, &lv_font_montserrat_28, LV_PART_MAIN);
        lv_obj_set_style_text_color(icon, lv_color_hex(0x7FB3FF), LV_PART_MAIN);

        lv_obj_t* title = lv_label_create(tile);
        lv_label_set_text(title, entry.title != nullptr ? entry.title : entry.id);
        lv_obj_set_style_text_color(title, lv_color_hex(0xD0D4D8), LV_PART_MAIN);
    }

    ESP_LOGI(TAG, "dung %u o", registry_->count());
}

void Launcher::onDestroy()
{
    // Nothing cached, so nothing to null. The tiles belong to the root, and the
    // host deletes the root -- rule 3. Listing that here rather than leaving the
    // method empty and unexplained.
}

}  // namespace ui
