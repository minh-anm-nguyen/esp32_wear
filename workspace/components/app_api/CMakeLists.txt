# The surface an APP is allowed to see. Nothing else.
#
# WHY THIS IS A SEPARATE COMPONENT FROM ui
#
# app.hpp rule 6 says an app never touches a driver, and the guide in
# doc-design/writing-an-app.md claimed the compiler enforced that. It did not.
#
# ESP-IDF propagates include directories through PUBLIC requirements, so while
# the apps depended on `ui` -- which must publicly require display and touch,
# because ui_manager.hpp includes both -- every app could reach
# display_manager.hpp and touch_manager.hpp without naming them. Proven by
# writing the bad #include and watching it compile.
#
# Splitting the contract out of the runtime closes that path: an app depends on
# app_api, app_api depends on lvgl and freertos, and there is no route from
# there to a driver header. The rule is now a build error rather than a review
# comment.
idf_component_register(SRCS "ui_context.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES lvgl freertos)
