idf_component_register(
    SRCS
        telemetry.c
    INCLUDE_DIRS
        include
    REQUIRES logging esp_wifi lwip
)
