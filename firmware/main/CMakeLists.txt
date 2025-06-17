idf_component_register(
    SRCS "main.c"
    # INCLUDE_DIRS "include"
    REQUIRES slam lidar
    PRIV_REQUIRES esp_driver_gpio
)
