message(STATUS "building uart interface mocks")

idf_component_get_property(original_uart_interface_dir uart_interface COMPONENT_OVERRIDEN_DIR)

set(include_dirs
    "${original_uart_interface_dir}/include"
)

idf_component_mock(INCLUDE_DIRS ${include_dirs}
    REQUIRES esp_common
    MOCK_HEADER_FILES ${original_uart_interface_dir}/include/uart_interface.h
)
