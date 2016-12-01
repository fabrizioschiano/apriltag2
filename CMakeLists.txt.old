project( apriltag2 )

cmake_minimum_required(VERSION 2.8)

# include_directories("/home/fschiano/Repositories/apriltag2")
# include_directories("/home/fschiano/Repositories/apriltag2/common")
# include_directories("/usr/include")

set(CMAKE_C_FLAGS "-std=gnu99 -fPIC -Wall -Wno-unused-parameter -Wno-unused-function -I. -O4")

message("CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}")

file(GLOB apriltag_SRC "*.c")
file(GLOB apriltag_HEADERS "*.h")

set(APRILTAG_SRCS ${apriltag_SRC})
set(APRILTAG_HEADERS ${apriltag_HEADERS})

message(STATUS "CMAKE_CURRENT_LIST_DIR=${CMAKE_CURRENT_LIST_DIR}")

add_library(apriltag STATIC ${APRILTAG_SRCS})

target_include_directories(apriltag PUBLIC ${CMAKE_SOURCE_DIR})
target_compile_options(apriltag PUBLIC -fPIC -Wall -Wno-unused-parameter -Wno-unused-function -O4)

install(TARGETS apriltag
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)
install(DIRECTORY CMAKE_CURRENT_LIST_DIR/include/
        DESTINATION CMAKE_CURRENT_LIST_DIR/include/
        FILES_MATCHING PATTERN *.h)
