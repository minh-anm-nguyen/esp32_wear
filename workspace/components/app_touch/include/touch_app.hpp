// Draws where you are touching.
//
// The simplest possible app, and therefore the template to copy: it reads the
// LVGL input device, needs no service, and depends on nothing but the contract
// in app.hpp. See doc-design/writing-an-app.md.
//
// It earned its place by settling touch::Geometry. The transform started as the
// identity, this app showed the marker landing at the point-mirror of the
// finger, and that pinned mirrorX + mirrorY (board.cpp). It stays because the
// same readout is how any future orientation question gets answered -- on the
// glass, without a serial monitor.
#pragma once

#include "app.hpp"

namespace apps {

class TouchApp final : public ui::AppBase {
public:
    const char* id() const override { return "touch"; }

    void onCreate(lv_obj_t* root) override;
    void onDestroy() override;
    void onTick(uint32_t nowMs) override;

private:
    lv_obj_t* root_{nullptr};
    lv_obj_t* readout_{nullptr};
    lv_obj_t* dot_{nullptr};
    lv_obj_t* hint_{nullptr};

    int32_t lastX_{-1};
    int32_t lastY_{-1};
    bool    lastPressed_{false};
};

}  // namespace apps
