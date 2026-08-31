# Crash attribution. No dependency on ui, on apps, or on LVGL.
#
# AppHost tells it which app is on screen; main.cpp asks it what the previous
# boot left behind. Neither direction requires this component to know anything
# about either of them, which is what lets a driver or a daemon use it too.
idf_component_register(SRCS "crash_log.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_system)
