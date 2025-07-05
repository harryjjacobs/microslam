cmake_minimum_required(VERSION 3.28)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)

project(microslam)

idf_build_set_property(COMPILE_OPTIONS "-DSLAM_LOG_DEBUG" APPEND)
