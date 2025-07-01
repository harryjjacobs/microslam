idf_component_register(
    SRCS
        "platform.c"
    INCLUDE_DIRS
        include
    REQUIRES logging slam uart_interface
)
