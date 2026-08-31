# Header only: shared vocabulary for the app/service layer, no translation unit
# of its own.
#
# topic.hpp is pure (bare g++ compiles it, host_test/ proves it).
# topic_esp.hpp adds the FreeRTOS critical-section policy, which is why
# freertos is required rather than merely available.
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES freertos)
