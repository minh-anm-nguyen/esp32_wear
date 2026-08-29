idf_component_register(
    SRCS "button.cpp" "button_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_timer
)
