project( apriltag2 C CXX)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_FLAGS "-std=gnu99 -fPIC -Wall -Wno-unused-parameter -Wno-unused-function -I. -O4")

include_directories("/home/fschiano/Repositories/apriltag2")
include_directories("/home/fschiano/Repositories/apriltag2/common")
include_directories("/usr/include")
add_library( apriltag STATIC apriltag.c )
add_library( apriltag SHARED apriltag.c )