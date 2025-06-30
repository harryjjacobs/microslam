set(SLAM_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../slam)

idf_component_register(
    SRCS 
        ${SLAM_SRC_DIR}/src/map.c
        ${SLAM_SRC_DIR}/src/scan_matching.c
        ${SLAM_SRC_DIR}/src/occupancy_quadtree.c
        ${SLAM_SRC_DIR}/src/utils.c
        ${SLAM_SRC_DIR}/src/scan.c
        system.c
    INCLUDE_DIRS
        include
        ${SLAM_SRC_DIR}/include
        ${SLAM_SRC_DIR}/thirdparty
    REQUIRES logging
)
