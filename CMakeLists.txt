cmake_minimum_required(VERSION 3.2)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(SOURCE_FILES src/main.cpp)

find_package(realsense2 REQUIRED)
include_directories("${realsense2_INCLUDE_DIR}")

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(scanner ${SOURCE_FILES})
target_link_libraries(scanner "${realsense2_LIBRARY}")
target_link_libraries(scanner ${PCL_LIBRARIES})
