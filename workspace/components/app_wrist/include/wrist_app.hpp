// Shows what the wrist is doing.
//
// The reference app, and the one that demonstrates BOTH halves:
//
//   this file          the UI half -- runs only while on screen
//   wrist_daemon.hpp   the background half -- runs from boot, app open or not
//
// The UI half reads two topics and renders them. No driver pointer, no service
// internals, no blocking -- app.hpp rules 5 and 6. Note that it reads its own
// daemon's topic exactly the same way it reads the service's: from the app's
// point of view a topic is a topic, and it does not care which side of the
// system filled it in.
#pragma once

#include "app.hpp"
#include "topic_esp.hpp"
#include "wrist_daemon.hpp"
#include "wrist_service.hpp"

namespace apps {

class WristApp final : public ui::AppBase {
public:
    using Topic = core::Topic<svc::WristState, core::CriticalSection>;

    WristApp(const Topic& topic, const WristDaemon::Topic& activity)
        : topic_(topic), activity_(activity)
    {
    }

    const char* id() const override { return "wrist"; }

    void onCreate(lv_obj_t* root) override;
    void onEnter() override;
    void onDestroy() override;
    void onTick(uint32_t nowMs) override;

private:
    void render(bool force);
    void renderActivity(bool force);

    const Topic&              topic_;
    const WristDaemon::Topic& activity_;

    // One cursor PER TOPIC. They advance independently, which is the whole
    // reason a cursor lives in the reader rather than in the topic.
    core::Cursor cursor_{};
    core::Cursor activityCursor_{};

    lv_obj_t* state_{nullptr};
    lv_obj_t* count_{nullptr};
    lv_obj_t* active_{nullptr};
};

}  // namespace apps
