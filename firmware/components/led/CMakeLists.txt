idf_component_register(
    SRCS
        "uart_interface.c"
    INCLUDE_DIRS
        include
    REQUIRES esp_driver_uart
)
