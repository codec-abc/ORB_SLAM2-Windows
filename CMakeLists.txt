cmake_minimum_required(VERSION 3.0)

project(orb_slam2_windows)

file(
  GLOB_RECURSE SRCS
  src/*.cc
  ThirdParty/g2o/g2o/core/*.cpp
  ThirdParty/g2o/g2o/stuff/*.c
  ThirdParty/g2o/g2o/stuff/*.cpp
  ThirdParty/g2o/g2o/types/*.cpp
  ThirdParty/DBow2/DBoW2/*.cpp
  ThirdParty/DBow2/DUtils/*.cpp
  windows_fix/*.cpp
)

set(OpenCVIncludeDir "" CACHE PATH "The include directory of the OpenCV directory")
set(OpenCVLibDir "" CACHE PATH "The library directory of the OpenCV directory")

include_directories("include")
include_directories("./")
include_directories("windows_fix")
include_directories("ThirdParty/g2o/g2o")
include_directories("ThirdParty")
include_directories("ThirdParty/DBow2/DBoW2")
include_directories("ThirdParty/DBow2/DUtils")

if (OpenCVIncludeDir STREQUAL "")
  message(FATAL_ERROR "OpenCV include directories not found")
else()
  include_directories(${OpenCVIncludeDir})
endif()

link_directories(${OpenCVLibDir})

add_library( orb_slam2_windows SHARED ${SRCS})

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} /bigobj")
target_link_libraries(orb_slam2_windows debug opencv_world331d optimized opencv_world331)