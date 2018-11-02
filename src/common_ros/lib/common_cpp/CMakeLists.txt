cmake_minimum_required(VERSION 2.8.3)
project(common_cpp)

## Add support for C++11
set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
  # Options: Debug, Release, MinSizeRel, RelWithDebInfo
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release")
endif()

## Enable openmp to speed up Eigen
set(CMAKE_CXX_FLAGS "-fopenmp")

## Find libraries
find_package(Eigen3 REQUIRED)
#find_package(OpenCV 3 REQUIRED)
#message(STATUS "OpenCV Version: ${OpenCV_VERSION} (${OpenCV_DIR})")

## Specify include directories
include_directories(include) 
#include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS}) 
