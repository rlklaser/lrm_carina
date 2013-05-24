cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#find_package(opende REQUIRED)
#include_directories(SYSTEM ${OPENDE_INCLUDE_DIRS})
#find_package(drawstuff REQUIRED)
#include_directories(SYSTEM ${DRAWSTUFF_INCLUDE_DIRS})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


rosbuild_add_executable(robots 
	src/bussola.cpp
	src/gps.cpp
	src/mymath.cpp
	src/obstaculo.cpp
	src/rna.c
	src/veiculo.cpp
#	src/antenna.cpp
	src/main.cpp
	)
target_link_libraries(robots ode drawstuff ${OPENGL_LIBRARIES})
#rosbuild_add_compile_flags(robots -msse4.2 -mpreferred-stack-boundary=4 -march=native -lomp)

rosbuild_add_executable(move_base_simple 
	src/move_base_simple.cpp
	)

#rosbuild_add_boost_directories()
#rosbuild_add_executable(multi_move_base_simple 
#	src/multi_move_base_simple.cpp
#	)
#rosbuild_link_boost(multi_move_base_simple signals)

rosbuild_add_executable(beacon_nav_node 
	src/beacon_nav_node.cpp
	)

rosbuild_add_executable(antenna_node 
	src/antenna_node.cpp
	)

#rosbuild_add_executable(aco 
#	src/bussola.cpp
#	src/gps.cpp
#	src/mymath.cpp
#	src/obstaculo.cpp
#	src/rna.c
#	src/veiculo.cpp
#	src/aco.cpp
#	src/main_aco.cpp
#	)
#target_link_libraries(aco ode drawstuff ${OPENGL_LIBRARIES})

