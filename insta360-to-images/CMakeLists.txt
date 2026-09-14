cmake_minimum_required(VERSION 3.16)
project(insta360_to_images CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra)

# Only panorama.cpp (decoding the camera's own firmware-stitched preview image)
# touches OpenCV; everything else in this project is dependency-free.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_library(insta360_core
  src/protobuf.cpp
  src/trailer.cpp
  src/metadata.cpp
  src/calibration.cpp
  src/sensors.cpp
  src/subprocess.cpp
  src/media.cpp
  src/panorama.cpp
  src/equirect.cpp
  src/convert.cpp
)
target_include_directories(insta360_core PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(insta360_core PUBLIC ${OpenCV_LIBS})

add_executable(insta360-to-images src/cli.cpp)
target_link_libraries(insta360-to-images PRIVATE insta360_core)

install(TARGETS insta360-to-images RUNTIME DESTINATION bin)

option(INSTA360_BUILD_TESTS "Build the unit test suite" ON)
if(INSTA360_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
