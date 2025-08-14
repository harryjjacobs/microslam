set(TESTS
    ${CMAKE_CURRENT_SOURCE_DIR}/test_ekf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_map.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_mat.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_occupancy_quadtree.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_scan_matching.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_serialisation.c
    ${CMAKE_CURRENT_SOURCE_DIR}/test_utils.c
)

foreach(TEST_SRC ${TESTS})
    get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
    target_link_libraries(${TEST_NAME} PRIVATE slam slam_viewer unity m glfw GL)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()
