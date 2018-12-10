cmake_minimum_required(VERSION 3.2)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

message("CMAKE_TOOLCHAIN_FILE: \"${CMAKE_TOOLCHAIN_FILE}\"")

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

if(realsense2_DLL)
	message(STATUS "Adding Post build script to copy realsense2.dll to project's binary folder")
	message(STATUS "Will try to copy ${realsense2_DLL} to ${CMAKE_CURRENT_BINARY_DIR}")
	add_custom_command(TARGET scanner POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			"${realsense2_DLL}"
			${CMAKE_CURRENT_BINARY_DIR})
endif()
