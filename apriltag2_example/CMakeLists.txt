# Author : Fabrizio SCHIANO

## Last update : Pol MORDEL 10/04/17
# add_dependecies for apriltag2_exe and clean the file

cmake_minimum_required(VERSION 2.8.3)
project(apriltag2_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(APRILTAG2_EXAMPLE_SRC_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(APRILTAG2_EXAMPLE_INCLUDE_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(APRILTAG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apriltag)
set(APRILTAG_COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apriltag-2016-12-01/common)

message("-->APRILTAG2_EXAMPLE_SRC_FOLDER: ${APRILTAG2_EXAMPLE_SRC_FOLDER}")
message("-->APRILTAG2_EXAMPLE_INCLUDE_FOLDER: ${APRILTAG2_EXAMPLE_INCLUDE_FOLDER}")

# Get the information about this package's buildtime dependencies
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
    message_filters
    )

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED
    COMPONENTS filesystem system signals regex date_time program_options thread)

# Declare the message files to be built
add_message_files(
    FILES
    AprilTagDetection.msg
    AprilTagDetectionArray.msg
    )

# Actually generate the language-specific message and service files
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    )

# Declare that this catkin package's runtime dependencies
catkin_package(
    INCLUDE_DIRS
    #include

    LIBRARIES

    CATKIN_DEPENDS
    image_transport
    roscpp
    sensor_msgs
    geometry_msgs
    message_filters
    tf
    cv_bridge
    message_runtime
    nodelet
    std_msgs

    DEPENDS
    Eigen3
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${APRILTAG2_EXAMPLE_INCLUDE_FOLDER}
    ${APRILTAG_DIR}
    ${APRILTAG_COMMON_DIR}
    )

# Make sure Boost.Filesystem v3 is used.
message("-->Verifying BOOST version...")
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)
message("-->Boost_INCLUDE_DIRS:${Boost_INCLUDE_DIRS}")
message("-->...done")


set( CMAKE_BUILD_TYPE Release)
set( CMAKE_BUILD_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/_build")

find_library(FLYCAPTURE2 flycapture)

find_library(APRILTAG_LIBRARY
	NAMES apriltag
	PATHS ${APRILTAG_DIR}
)

message("-->APRILTAG_LIBRARY: ${APRILTAG_LIBRARY}")

# Needed libraries
set (EXTRA_LIBS ${APRILTAG_LIBRARY} flycapture ${OpenCV_LIBS} ${VISP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Define executable apriltag2_exe using messages provided by this package.
add_executable(apriltag2_exe
    ${APRILTAG2_EXAMPLE_SRC_FOLDER}/apriltag2_pub_sub.cpp
    ${APRILTAG2_EXAMPLE_SRC_FOLDER}/node.cpp
    ${APRILTAG2_EXAMPLE_SRC_FOLDER}/node.h
    ${APRILTAG2_EXAMPLE_SRC_FOLDER}/names.cpp
    ${APRILTAG2_EXAMPLE_SRC_FOLDER}/names.h
    )

add_dependencies(apriltag2_exe ${${apriltag2_exe}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# specify which libraries apriltag2_exe links against
target_link_libraries(apriltag2_exe
    ${EXTRA_LIBS}
    )
