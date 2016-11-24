# Author

cmake_minimum_required(VERSION 2.8)
project( opencvAprilTagFlea )
find_package( OpenCV REQUIRED )
include_directories("/home/fschiano/Repositories/apriltag2")
include_directories("/home/fschiano/Repositories/apriltag2/common")
find_library(FLYCAPTURE2 flycapture)
add_executable( opencvDemo opencv_demo.cc )

# Needed libraries
set (EXTRA_LIBS ${EXTRA_LIBS} apriltag flycapture ${OpenCV_LIBS})

# target_link_libraries(<target> [item1 [item2 [...]]]
#                       [[debug|optimized|general] <item>] ...)
# Specify libraries or flags to use when linking a given target. 
# The named <target> must have been created in the current directory by a 
# command such as add_executable() or add_library(). 
# The remaining arguments specify library names or flags. 
# Repeated calls for the same <target> append items in the order called.
target_link_libraries( opencvDemo ${EXTRA_LIBS} )
