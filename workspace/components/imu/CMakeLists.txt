idf_component_register(
    SRCS "qmi8658.cpp" "axis_remap.cpp" "imu_manager.cpp" "axis_calibration.cpp"
    INCLUDE_DIRS "include"
    REQUIRES i2c_bus sensor_types esp_driver_gpio esp_timer
)
