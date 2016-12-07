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
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

set( CMAKE_BUILD_TYPE Release)
set( CMAKE_BUILD_DIRECTORY "/home/fschiano/Repositories/apriltag2/apriltag2_example/_build")

#DEBUG
message( "\nCMAKE_BUILD_DIRECTORY=\n${CMAKE_BUILD_DIRECTORY}\n")

find_package( OpenCV REQUIRED )
find_package(VISP REQUIRED)

include_directories("/home/fschiano/Repositories/apriltag2")
include_directories("/home/fschiano/Repositories/apriltag2/common")
include_directories("usr/local")
find_library(FLYCAPTURE2 flycapture)
add_executable( opencvDemo_exe opencv_demo.cc )

include_directories(${VISP_INCLUDE_DIRS})

#DEBUG
message( "VISP_INCLUDE_DIRS=\n${VISP_INCLUDE_DIRS}\n")
message( "VISP_LIBRARIES=\n${VISP_LIBRARIES}\n")

# Needed libraries
set (EXTRA_LIBS apriltag flycapture ${OpenCV_LIBS} ${VISP_LIBRARIES})
target_link_libraries( opencvDemo_exe ${EXTRA_LIBS} )