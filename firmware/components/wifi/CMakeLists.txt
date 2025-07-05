idf_component_register(
    SRCS
        wifi.c
    INCLUDE_DIRS
        include
    REQUIRES logging esp_wifi
    PRIV_REQUIRES nvs_flash
)
