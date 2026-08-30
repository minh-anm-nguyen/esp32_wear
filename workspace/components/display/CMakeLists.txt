# display.cpp is the pure layer and pulls in nothing from ESP-IDF -- that is what
# lets host_test/ build with a bare g++. Everything below it is the driver half.
#
# No REQUIRES on lvgl, and there must never be one: components/ui is the only
# place that knows about both. See doc-design/display.md section 15.
idf_component_register(
    SRCS "display.cpp" "display_manager.cpp" "display_selftest.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_lcd esp_driver_spi esp_driver_ledc esp_driver_gpio esp_timer
)
