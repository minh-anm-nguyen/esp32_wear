#include "touch_app.hpp"

#include <cstdio>

namespace apps {

void TouchApp::onCreate(lv_obj_t* root)
{
    root_ = root;
    lv_obj_set_style_bg_color(root, lv_color_hex(0x0E1116), LV_PART_MAIN);
    lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(root, LV_SCROLLBAR_MODE_OFF);

    readout_ = lv_label_create(root);
    lv_obj_set_style_text_font(readout_, &lv_font_montserrat_28, LV_PART_MAIN);
    lv_obj_set_style_text_color(readout_, lv_color_hex(0xE8E8E8), LV_PART_MAIN);
    lv_label_set_text(readout_, "--");
    lv_obj_align(readout_, LV_ALIGN_TOP_MID, 0, 26);

    hint_ = lv_label_create(root);
    lv_obj_set_style_text_color(hint_, lv_color_hex(0x9AA0A6), LV_PART_MAIN);
    lv_label_set_text(hint_, "cham 4 goc");
    lv_obj_align(hint_, LV_ALIGN_TOP_MID, 0, 62);

    // A small marker that follows the finger. If it lands on the OPPOSITE
    // corner from the one being touched, that is the mirror the identity
    // transform is expected to have -- and which flag to flip is then obvious.
    dot_ = lv_obj_create(root);
    lv_obj_set_size(dot_, 18, 18);
    lv_obj_set_style_radius(dot_, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_color(dot_, lv_color_hex(0xFF6B6B), LV_PART_MAIN);
    lv_obj_set_style_border_width(dot_, 0, LV_PART_MAIN);
    lv_obj_add_flag(dot_, LV_OBJ_FLAG_HIDDEN);
}

void TouchApp::onDestroy()
{
    root_    = nullptr;
    readout_ = nullptr;
    dot_     = nullptr;
    hint_    = nullptr;
}

void TouchApp::onTick(uint32_t)
{
    if (readout_ == nullptr) {
        return;
    }

    // Asking LVGL rather than the touch driver: popTransition() has EXACTLY one
    // consumer and it is the input callback (trap #18). An app that popped it
    // too would steal events from LVGL and leave touches stuck down.
    lv_indev_t* indev = lv_indev_get_next(nullptr);
    if (indev == nullptr) {
        return;
    }

    lv_point_t p{};
    lv_indev_get_point(indev, &p);
    const bool pressed = (lv_indev_get_state(indev) == LV_INDEV_STATE_PRESSED);

    if (p.x == lastX_ && p.y == lastY_ && pressed == lastPressed_) {
        return;   // nothing moved: do not dirty the screen (trap #19)
    }
    lastX_       = p.x;
    lastY_       = p.y;
    lastPressed_ = pressed;

    char line[32];
    std::snprintf(line, sizeof(line), "%ld,%ld", static_cast<long>(p.x),
                  static_cast<long>(p.y));
    lv_label_set_text(readout_, line);

    if (pressed) {
        lv_obj_clear_flag(dot_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(dot_, p.x - 9, p.y - 9);
    } else {
        lv_obj_add_flag(dot_, LV_OBJ_FLAG_HIDDEN);
    }
}

}  // namespace apps
