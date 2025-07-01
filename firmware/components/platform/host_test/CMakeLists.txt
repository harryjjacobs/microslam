cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    ${CMAKE_SOURCE_DIR}/../../../mocks/uart_interface
    ${CMAKE_SOURCE_DIR}/../../platform
    # $ENV{IDF_PATH}/tools/mocks/freertos/ # example of how to include a mock ESP-IDF component
)


set(COMPONENT_DIRS 
    $ENV{IDF_PATH}/components
    ${CMAKE_CURRENT_LIST_DIR}/main
    # Explicitly include the component directories required by the platform component
    ${CMAKE_SOURCE_DIR}/../../logging
    ${CMAKE_SOURCE_DIR}/../../uart_interface
    ${CMAKE_SOURCE_DIR}/../../slam
    # Add the extra component directories last to ensure any mocked components override the originals
    ${EXTRA_COMPONENT_DIRS}
)

message(STATUS "Component dirs: ${COMPONENT_DIRS}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(COMPONENTS main)

project(platform_test)

function(enable_address_sanitizer)
    message(STATUS "Enabling AddressSanitizer")
    idf_build_set_property(COMPILE_OPTIONS "-fsanitize=address" APPEND)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address -lasan" PARENT_SCOPE)
endfunction()

# enable_address_sanitizer()

message(STATUS "Component dirs: ${COMPONENT_DIRS}")
