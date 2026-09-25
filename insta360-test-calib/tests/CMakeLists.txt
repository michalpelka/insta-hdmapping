add_executable(calib_core_tests
    test_main.cpp
    calibration_test.cpp
    camera_test.cpp
    io_test.cpp
)
target_link_libraries(calib_core_tests PRIVATE calib_core)
target_include_directories(calib_core_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_test(NAME calib_core_tests COMMAND calib_core_tests)
