# Note: sensor_types only. NOT imu -- this component must never learn which
# chip produced the samples. See doc-design/imu-qmi8658c-design.md section 18.
idf_component_register(
    SRCS "motion_controller.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sensor_types
)
