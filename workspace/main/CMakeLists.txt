idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES button buzzer i2c_bus sensor_types imu motion display)
