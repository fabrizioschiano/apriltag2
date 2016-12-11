# Author

cmake_minimum_required(VERSION 2.8.3)
project(apriltag2_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   sensor_msgs
#   geometry_msgs
#   tf
#   cmake_modules
# )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
# )


find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  cv_bridge
  message_generation
  nodelet
  std_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package( OpenCV REQUIRED )
find_package(VISP REQUIRED)

add_message_files(
  FILES
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES apriltag_detector
  CATKIN_DEPENDS image_transport roscpp sensor_msgs geometry_msgs tf cv_bridge message_runtime nodelet std_msgs
  DEPENDS Eigen
)


set( CMAKE_BUILD_TYPE Release)
set( CMAKE_BUILD_DIRECTORY "/home/fschiano/Repositories/apriltag2/apriltag2_example/_build")

#DEBUG
message( "-->CMAKE_BUILD_DIRECTORY=\n${CMAKE_BUILD_DIRECTORY}\n")
message("--> include_directories...\n")
message("--> catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message("--> Eigen_INCLUDE_DIRS: ${Eigen_INCLUDE_DIRS}")
#include_directories(include)
include_directories("/home/fschiano/Repositories/apriltag2")
include_directories("/home/fschiano/Repositories/apriltag2/common")
#include_directories("usr/local")
find_library(FLYCAPTURE2 flycapture)
add_executable( opencvDemo_exe opencv_demo.cc )
add_executable( test_sub test_subscriber.cpp )

include_directories(${VISP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
#DEBUG
message( "-->VISP_INCLUDE_DIRS= ${VISP_INCLUDE_DIRS}")
message( "-->VISP_LIBRARIES= ${VISP_LIBRARIES}")
message("-->CMAKE_BINARY_DIR= ${CMAKE_BINARY_DIR}")
message("-->CMAKE_CURRENT_SOURCE_DIR= ${CMAKE_CURRENT_SOURCE_DIR}")

set(apriltagSharedLibrary ${CMAKE_CURRENT_SOURCE_DIR}/lib/libapriltag.so)
message("-->apriltagSharedLibrary: ${apriltagSharedLibrary}")

# Needed libraries
set (EXTRA_LIBS ${apriltagSharedLibrary} flycapture ${OpenCV_LIBS} ${VISP_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries( opencvDemo_exe ${EXTRA_LIBS})
target_link_libraries( test_sub ${EXTRA_LIBS})


# 
