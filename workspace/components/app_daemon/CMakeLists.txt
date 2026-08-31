# The daemon runtime. Deliberately does NOT require lvgl.
#
# A daemon runs in its own task, where LVGL's single-threaded invariant has no
# lock protecting it. Keeping this component off the LVGL dependency graph means
# the daemon runtime itself can never grow a dependency on it -- the same
# structural move that split app_api out of ui.
idf_component_register(SRCS "daemon_host.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES sensor_types freertos esp_timer esp_system)
