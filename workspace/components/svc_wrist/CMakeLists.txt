# The service tier. Header only for now: WristServiceT is a template, because
# the lock policy is what separates the host-testable logic from the FreeRTOS
# critical section (see topic.hpp).
#
# REQUIRES, not PRIV_REQUIRES: wrist_service.hpp includes all three in its
# public interface.
idf_component_register(INCLUDE_DIRS "include"
                       REQUIRES app_core motion sensor_types)
