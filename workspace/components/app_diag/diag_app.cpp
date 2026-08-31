#include "diag_app.hpp"

#include <cstdio>

namespace apps {
namespace {
// Render slowly, on purpose.
//
// A diagnostics screen that updates at the frame rate ends up measuring the
// load it is itself creating, and reports a frame rate and a heap figure that
// nothing else in the system would ever see. Twice a second is faster than
// anyone can read anyway.
constexpr uint32_t kRenderPeriodMs = 500;
}  // namespace

void DiagApp::onCreate(lv_obj_t* root)
{
    lv_obj_set_style_bg_color(root, lv_color_hex(0x101014), LV_PART_MAIN);
    lv_obj_set_style_border_width(root, 0, LV_PART_MAIN);

    lv_obj_t* title = lv_label_create(root);
    lv_label_set_text(title, "Chan doan");
    lv_obj_set_style_text_color(title, lv_color_hex(0x7FB3FF), LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 16);

    body_ = lv_label_create(root);
    lv_obj_set_style_text_color(body_, lv_color_hex(0xD0D4D8), LV_PART_MAIN);
    lv_label_set_text(body_, "...");
    lv_obj_align(body_, LV_ALIGN_TOP_LEFT, 12, 48);
}

void DiagApp::onEnter()
{
    lastRenderMs_ = 0;   // render immediately on entry rather than after 500 ms
}

void DiagApp::onDestroy() { body_ = nullptr; }

void DiagApp::onTick(uint32_t nowMs)
{
    if (body_ == nullptr || fn_ == nullptr) {
        return;
    }
    if (lastRenderMs_ != 0 && (nowMs - lastRenderMs_) < kRenderPeriodMs) {
        return;
    }
    lastRenderMs_ = nowMs;

    DiagSnapshot s{};
    fn_(ctx_, s);

    char text[256];
    std::snprintf(text, sizeof(text),
                  "khung: %u\n"
                  "flush/khung: %u.%02u\n"
                  "can bang: %s\n\n"
                  "stack con: %u B\n\n"
                  "LVGL con: %u B\n"
                  "  manh lon: %u B\n\n"
                  "SRAM con: %u B\n"
                  "  manh lon: %u B",
                  static_cast<unsigned>(s.frames),
                  static_cast<unsigned>(s.flushPerFrame100 / 100),
                  static_cast<unsigned>(s.flushPerFrame100 % 100),
                  s.flushBalanced ? "CO" : "KHONG",
                  static_cast<unsigned>(s.stackFreeBytes),
                  static_cast<unsigned>(s.lvglFreeBytes),
                  static_cast<unsigned>(s.lvglLargestBytes),
                  static_cast<unsigned>(s.sramFreeBytes),
                  static_cast<unsigned>(s.sramLargestBytes));

    lv_label_set_text(body_, text);
}

}  // namespace apps
