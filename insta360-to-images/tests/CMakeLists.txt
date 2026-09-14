add_executable(insta360_tests
  test_main.cpp
  protobuf_test.cpp
  trailer_test.cpp
  calibration_test.cpp
  sensors_test.cpp
  convert_test.cpp
  panorama_test.cpp
  equirect_test.cpp
)
target_link_libraries(insta360_tests PRIVATE insta360_core)
target_include_directories(insta360_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_test(NAME unit_tests COMMAND insta360_tests)
