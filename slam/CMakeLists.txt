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
    src/default_logging.c
    src/serialisation.c
    thirdparty/log/log.c
)
target_include_directories(slam PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(slam PRIVATE SLAM_LOG_DEBUG)
else()
    target_compile_definitions(slam PRIVATE SLAM_LOG_INFO)
endif()

# === Viewer library ===

add_library(slam_viewer STATIC 
    src/viewer.c
)
target_include_directories(slam_viewer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
target_link_libraries(slam_viewer PRIVATE slam m glfw GL)

# === Test app executable ===

add_executable(slam_test_app
    src/main.c
)
target_link_libraries(slam_test_app PRIVATE slam slam_viewer)
target_compile_definitions(slam_test_app PRIVATE SLAM_LOG_INFO)

# === Unity test object ===

add_library(unity STATIC 
    thirdparty/unity/unity.c
)
target_include_directories(unity PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)

# === Tests ===

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
    if (BUILD_TESTING)
        add_subdirectory(test)
    endif()
endif()

# === Aliases for libraries === #
add_library(slam::slam ALIAS slam)
add_library(slam::slam_viewer ALIAS slam_viewer)
