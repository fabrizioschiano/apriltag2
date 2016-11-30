# Author

cmake_minimum_required(VERSION 2.8)
project( opencv_apriltag2_flea )

set( CMAKE_BUILD_TYPE Release)
set( CMAKE_BUILD_DIRECTORY "/home/fschiano/Repositories/apriltag2/example")

#DEBUG
message( "\nCMAKE_BUILD_DIRECTORY=\n${CMAKE_BUILD_DIRECTORY}\n")

find_package( OpenCV REQUIRED )
find_package(VISP REQUIRED)

include_directories("/home/fschiano/Repositories/apriltag2")
include_directories("/home/fschiano/Repositories/apriltag2/common")
include_directories("usr/local")
find_library(FLYCAPTURE2 flycapture)
add_executable( opencvDemo opencv_demo.cc )

include_directories(${VISP_INCLUDE_DIRS})

#DEBUG
message( "VISP_INCLUDE_DIRS=\n${VISP_INCLUDE_DIRS}\n")
message( "VISP_LIBRARIES=\n${VISP_LIBRARIES}\n")

# Needed libraries
set (EXTRA_LIBS apriltag flycapture ${OpenCV_LIBS} ${VISP_LIBRARIES})
target_link_libraries( opencvDemo ${EXTRA_LIBS} )
