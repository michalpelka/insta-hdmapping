cmake_minimum_required(VERSION 3.20)
project(insta3360-to-images CXX)

# One-shot build/install of every subproject:
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build -j
#   cmake --install build
#
# Each subproject also builds standalone (see its own CMakeLists.txt) — CI
# uses that path so a missing dependency for one doesn't block the other.

option(INSTA360_BUILD_TO_IMAGES "Build insta360-to-images (.insv unpacker)" ON)
option(INSTA360_BUILD_TEST_CALIB "Build insta360-test-calib (calibration GUI)" ON)

if(INSTA360_BUILD_TO_IMAGES)
  add_subdirectory(insta360-to-images)
endif()

if(INSTA360_BUILD_TEST_CALIB)
  add_subdirectory(insta360-test-calib)
endif()
