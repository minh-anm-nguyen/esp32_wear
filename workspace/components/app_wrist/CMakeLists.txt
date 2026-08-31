# The reference app, and the one that carries a DAEMON.
#
# Two halves in one component, which is the whole point of the arrangement: the
# UI half runs only while the app is on screen, the daemon half attaches to the
# IMU stream at boot and keeps working while the app is closed. A developer adds
# both by adding one folder.
#
# sensor_types is here for the daemon's ISampleSink; svc_wrist for the topic the
# UI half reads. NO driver, ever -- app.hpp rule 6.
idf_component_register(SRCS "wrist_app.cpp" "wrist_daemon.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES app_api app_daemon app_core svc_wrist sensor_types)
