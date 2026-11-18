cmake_minimum_required(VERSION 3.16)
project(insta360_to_images CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra)

# No third-party libraries: frames come out of ffmpeg already JPEG-encoded, and
# everything else is parsed by hand.
add_library(insta360_core
  src/protobuf.cpp
  src/trailer.cpp
  src/metadata.cpp
  src/calibration.cpp
  src/sensors.cpp
  src/subprocess.cpp
  src/media.cpp
  src/imu_align.cpp
  src/convert.cpp
)
target_include_directories(insta360_core PUBLIC include)

add_executable(insta360-to-images src/cli.cpp)
target_link_libraries(insta360-to-images PRIVATE insta360_core)

install(TARGETS insta360-to-images RUNTIME DESTINATION bin)

option(INSTA360_BUILD_TESTS "Build the unit test suite" ON)
if(INSTA360_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
