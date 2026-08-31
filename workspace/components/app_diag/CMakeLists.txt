# Health of the UI runtime, as an app. Takes its numbers through a callback, so
# it depends on the contract and not on UiManager.
idf_component_register(SRCS "diag_app.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES app_api)
