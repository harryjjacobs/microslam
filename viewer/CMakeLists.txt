cmake_minimum_required(VERSION 3.28)

project(microslam_viewer)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  slam
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/../slam
)
FetchContent_MakeAvailable(slam)

add_executable(microslam_viewer
    src/main.c
    src/net.c
)
target_include_directories(microslam_viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(microslam_viewer PRIVATE slam::slam_viewer)
