cmake_minimum_required(VERSION 2.8.3)
project(common_ros)

## Add support for C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
  # Options: Debug, Release, MinSizeRel, RelWithDebInfo
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release")
endif()

## Enable openmp to speed up Eigen
set(CMAKE_CXX_FLAGS "-fopenmp")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Version: ${OpenCV_VERSION} (${OpenCV_DIR})")

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include lib/common_cpp/include
  LIBRARIES common_ros common_cpp
  CATKIN_DEPENDS roscpp std_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## declare cpp libraries 
include_directories(lib/common_cpp/include) 
add_library(common_cpp lib/common_cpp/src/common.cpp)

## Specify libraries to link a library or executable target against
add_library(common_ros src/common_ros.cpp)
target_link_libraries(common_ros common_cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS common_ros
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN "*.svn" EXCLUDE
)
