idf_component_register(
    SRCS "buzzer.cpp" "buzzer_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_ledc esp_timer
)
