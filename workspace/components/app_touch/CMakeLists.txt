# One app, one component: its own folder, its own build unit, its own header.
# Depends on ui for the IApp contract and on nothing else -- this app reads the
# input device and nothing more.
idf_component_register(SRCS "touch_app.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES app_api)
