idf_component_register(
    SRCS "cst816t.cpp" "touch_tracker.cpp" "touch_transform.cpp" "touch_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES i2c_bus esp_driver_gpio esp_timer
)
