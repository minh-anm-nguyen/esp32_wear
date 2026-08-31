idf_component_register(SRCS "main.cpp" "board.cpp" "services.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES button buzzer i2c_bus sensor_types app_core crash_log
                             svc_wrist imu motion display touch ui app_daemon app_wrist app_touch app_diag)
