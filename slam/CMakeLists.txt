cmake_minimum_required(VERSION 3.28)

project(slam C)

# === Build type handling ===

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# === Compiler flags ===

set(COMMON_CFLAGS "-Wall -Wextra")
set(PROFILE_FLAGS "-pg -O3")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_C_FLAGS "${COMMON_CFLAGS} -g")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_C_FLAGS "${COMMON_CFLAGS} -O3")
elseif(CMAKE_BUILD_TYPE STREQUAL "Profile")
    set(CMAKE_C_FLAGS "${COMMON_CFLAGS} ${PROFILE_FLAGS}")
else()
    message(FATAL_ERROR "Unsupported build type: ${CMAKE_BUILD_TYPE}")
endif()

# === Library ===

add_library(slam STATIC 
    src/map.c
    src/occupancy_quadtree.c
    src/scan_matching.c
    src/scan.c
    src/utils.c
    thirdparty/log/log.c
)
target_include_directories(slam PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)

# === Viewer library ===

add_library(microslam_viewer STATIC 
    src/microslam_viewer.c
)
target_include_directories(microslam_viewer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
target_link_libraries(microslam_viewer PRIVATE slam m glfw GL)

# === Test app executable ===

add_executable(slam_test_app
    src/main.c
)
target_link_libraries(slam_test_app PRIVATE slam microslam_viewer)

# === Unity test object ===

add_library(unity STATIC 
    thirdparty/unity/unity.c
)
target_include_directories(unity PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)

# === Tests ===

set(TESTS
    test/test_map.c
    test/test_occupancy_quadtree.c
    test/test_scan_matching.c
    test/test_utils.c
)

foreach(TEST_SRC ${TESTS})
    get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)
    add_executable(test_${TEST_NAME} ${TEST_SRC})
    target_include_directories(test_${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
    target_link_libraries(test_${TEST_NAME} PRIVATE slam microslam_viewer unity m glfw GL)
    add_test(NAME ${TEST_NAME} COMMAND test_${TEST_NAME})
endforeach()
