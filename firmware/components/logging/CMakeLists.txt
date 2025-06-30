set(SLAM_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../slam)

idf_component_register(
    SRCS 
        logging.c
    INCLUDE_DIRS
        include
        ${SLAM_SRC_DIR}/include
)
