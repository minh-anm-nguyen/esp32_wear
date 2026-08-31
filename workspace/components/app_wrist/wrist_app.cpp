#include "wrist_app.hpp"

#include <cstdio>

namespace apps {

void WristApp::onCreate(lv_obj_t* root)
{
    lv_obj_set_style_bg_color(root, lv_color_hex(0x101014), LV_PART_MAIN);
    lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);

    lv_obj_t* title = lv_label_create(root);
    lv_label_set_text(title, "Co tay");
    lv_obj_set_style_text_color(title, lv_color_hex(0x7FB3FF), LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    state_ = lv_label_create(root);
    lv_obj_set_style_text_font(state_, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(state_, lv_color_hex(0xE8E8E8), LV_PART_MAIN);
    lv_label_set_text(state_, "--");
    lv_obj_align(state_, LV_ALIGN_CENTER, 0, -10);

    count_ = lv_label_create(root);
    lv_obj_set_style_text_color(count_, lv_color_hex(0x9AA0A6), LV_PART_MAIN);
    lv_label_set_text(count_, "");
    lv_obj_align(count_, LV_ALIGN_CENTER, 0, 30);

    // Filled from the DAEMON's topic. It keeps counting while this app is
    // closed, so the number jumps forward between visits -- which is the whole
    // point of the arrangement being visible on screen.
    active_ = lv_label_create(root);
    lv_obj_set_style_text_color(active_, lv_color_hex(0x7CE38B), LV_PART_MAIN);
    lv_label_set_text(active_, "");
    lv_obj_align(active_, LV_ALIGN_BOTTOM_MID, 0, -28);
}

void WristApp::onEnter()
{
    // A FRESH cursor on every entry, and then one forced render.
    //
    // Without the reset, an app re-entered later would compare against a
    // generation from its previous visit and correctly conclude "nothing new" --
    // leaving the screen showing the placeholder until the wrist next moved.
    // Rule 5: subscribe on enter.
    cursor_         = core::Cursor{};
    activityCursor_ = core::Cursor{};
    render(true);
    renderActivity(true);
}

void WristApp::onDestroy()
{
    // Rule 3: the host is about to delete the root, which takes every child
    // with it. Forget them, or the next onTick writes into freed memory.
    state_  = nullptr;
    count_  = nullptr;
    active_ = nullptr;
}

void WristApp::onTick(uint32_t)
{
    render(false);
    renderActivity(false);
}

void WristApp::render(bool force)
{
    if (state_ == nullptr) {
        return;
    }

    svc::WristState st{};
    const bool      fresh = topic_.read(st, cursor_);
    if (!fresh && !force) {
        return;   // nothing changed: do not touch LVGL, do not dirty the screen
    }
    if (!fresh && !topic_.peek(st)) {
        lv_label_set_text(state_, "--");
        lv_label_set_text(count_, "chua co du lieu IMU");
        return;
    }

    lv_label_set_text(state_, st.raised ? "NANG" : "ha");
    lv_obj_set_style_text_color(
        state_, st.raised ? lv_color_hex(0x7CE38B) : lv_color_hex(0xE8E8E8),
        LV_PART_MAIN);

    char line[32];
    std::snprintf(line, sizeof(line), "lan thu %u", static_cast<unsigned>(st.raiseCount));
    lv_label_set_text(count_, line);
}

void WristApp::renderActivity(bool force)
{
    if (active_ == nullptr) {
        return;
    }

    ActivityState a{};
    const bool    fresh = activity_.read(a, activityCursor_);
    if (!fresh && !force) {
        return;
    }
    if (!fresh && !activity_.peek(a)) {
        return;
    }

    char line[48];
    std::snprintf(line, sizeof(line), "hoat dong %u giay", 
                  static_cast<unsigned>(a.activeMs / 1000));
    lv_label_set_text(active_, line);
}

}  // namespace apps
