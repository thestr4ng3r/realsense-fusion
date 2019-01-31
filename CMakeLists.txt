cmake_minimum_required(VERSION 3.2)

option(BUILD_INPUT_REALSENSE "Build with RealSense support" ON)
option(BUILD_INPUT_KINECT "Build with Kinect support" OFF)

option(BUILD_TESTS "Build test executables" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

message("CMAKE_TOOLCHAIN_FILE: \"${CMAKE_TOOLCHAIN_FILE}\"")

add_subdirectory(third-party)

set(HEADER_FILES
		include/frame.h
		include/input.h
		include/model.h
		include/marching_cubes.h
		include/mesh.h
		include/realsense_input.h
		include/renderer.h
		include/window.h
		include/gl_model.h
		include/pc_integrator.h
		include/shader_common.h
		include/camera_transform.h
		include/icp.h)

set(SOURCE_FILES
		src/realsense_input.cpp
		src/frame.cpp
		src/model.cpp
		src/marching_cubes.cpp
		src/renderer.cpp
		src/window.cpp
		src/gl_model.cpp
		src/pc_integrator.cpp
		src/shader_common.cpp
		src/camera_transform.cpp
		src/icp.cpp)

set(SOURCE_FILE_MAIN
		src/main.cpp)

set(MODEL_TEST_FILES
		tests/marchingcubestest.cpp
		src/marching_cubes.cpp
		src/model.cpp)


include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

if(BUILD_INPUT_REALSENSE)
	find_package(realsense2 REQUIRED)
	include_directories("${realsense2_INCLUDE_DIR}")
	add_definitions(-DENABLE_INPUT_REALSENSE)
endif()

if(BUILD_INPUT_KINECT)
	# TODO: find_package(...) for kinect
	add_definitions(-DENABLE_INPUT_KINECT)
endif()

find_package(glfw3 REQUIRED)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GLEW)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/third-party/imgui")

add_executable(scanner ${SOURCE_FILES} ${HEADER_FILES} ${SOURCE_FILE_MAIN} ${IMGUI_SOURCE_FILES})
target_link_libraries(scanner ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} glfw Eigen3::Eigen)

if(BUILD_TESTS)
	add_executable(modeltest ${MODEL_TEST_FILES})
	target_link_libraries(modeltest Eigen3::Eigen)

	add_executable(integrationtest ${SOURCE_FILES} ${HEADER_FILES} tests/integrationtest.cpp ${IMGUI_SOURCE_FILES})
	target_link_libraries(integrationtest ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} glfw Eigen3::Eigen)

	add_executable(marchingcubestest ${SOURCE_FILES} ${HEADER_FILES} tests/marchingcubestest.cpp ${IMGUI_SOURCE_FILES})
	target_link_libraries(marchingcubestest ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} glfw Eigen3::Eigen)
endif()



if(BUILD_INPUT_REALSENSE)
	target_link_libraries(scanner "${realsense2_LIBRARY}")
	if(BUILD_TESTS)
		#target_link_libraries(modeltest "${realsense2_LIBRARY}")
		target_link_libraries(renderertest "${realsense2_LIBRARY}")
		target_link_libraries(integrationtest "${realsense2_LIBRARY}")
		target_link_libraries(marchingcubestest "${realsense2_LIBRARY}")
	endif()
	if(realsense2_DLL)
		message(STATUS "Adding Post build script to copy realsense2.dll to project's binary folder")
		message(STATUS "Will try to copy ${realsense2_DLL} to ${CMAKE_CURRENT_BINARY_DIR}")
		add_custom_command(TARGET scanner POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy_if_different
				"${realsense2_DLL}"
				${CMAKE_CURRENT_BINARY_DIR})
	endif()
endif()

if(BUILD_INPUT_KINECT)
	# TODO: target_link_libraries(scanner ...) for kinect
endif()
