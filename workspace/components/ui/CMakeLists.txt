# The LVGL runtime. The ONLY component that links against lvgl, and the only
# place LVGL calls are legal -- and then only from the UI task.
#
# No arrow from display or touch to lvgl: both stay compilable by a bare g++ at
# their pure layer, and this is where the two sides meet. display.md section 15.
idf_component_register(SRCS "ui_manager.cpp" "app_host.cpp" "launcher.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES app_api app_daemon crash_log display touch lvgl app_core esp_timer)
