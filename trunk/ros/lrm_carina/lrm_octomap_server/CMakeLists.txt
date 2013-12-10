cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#set(CMAKE_CXX_COMPILER "/usr/bin/g++")
#set(CMAKE_C_COMPILER "/usr/bin/gcc")

#set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
#set(CMAKE_C_COMPILER "/usr/bin/clang")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

#set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)

add_definitions(-std=c++0x)
add_definitions(-msse3 -msse4.1 -msse4.2)
add_definitions(-mfpmath=sse -Ofast -flto -march=native)

add_definitions(-DEIGEN_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

rosbuild_init()

#find_package(octomap REQUIRED)
#include_directories(${OCTOMAP_INCLUDE_DIRS})
#link_directories(${OCTOMAP_LIBRARY_DIRS})
#add_definitions(-DOCTOMAP_NODEBUGOUT)

#add_definitions(-DEIGEN_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})

#find_package(nodelet REQUIRED)
#include_directories(${nodelet_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} 
	src/OctomapServer.cpp 
	src/conversions.cpp
	#src/OctomapServerMultilayer.cpp 
	#src/TrackingOctomapServer.cpp
	)

#target_link_libraries(
#  ${PROJECT_NAME}
#  ${PCL_COMMON_LIBRARIES}
#  ${OCTOMAP_LIBRARIES}
#  ${nodelet_LIBRARIES}
#)

rosbuild_add_executable(octomap_server_node src/octomap_server_node.cpp)
target_link_libraries(octomap_server_node ${PROJECT_NAME})# ${OCTOMAP_LIBRARIES})

#rosbuild_add_compile_flags(octomap_server_node -msse4.2 -mpreferred-stack-boundary=4 -march=native)

#rosbuild_add_executable(octomap_server_multilayer src/octomap_server_multilayer.cpp)
#target_link_libraries(octomap_server_multilayer ${PROJECT_NAME} ${OCTOMAP_LIBRARIES})

rosbuild_add_executable(octomap_saver src/octomap_saver.cpp)
target_link_libraries(octomap_saver ${PROJECT_NAME})# ${OCTOMAP_LIBRARIES})

#rosbuild_add_executable(octomap_tracking_server_node src/octomap_tracking_server_node.cpp)
#target_link_libraries(octomap_tracking_server_node ${PROJECT_NAME} ${OCTOMAP_LIBRARIES})

# Nodelet
rosbuild_add_library(octomap_server_nodelet src/octomap_server_nodelet.cpp)
target_link_libraries(octomap_server_nodelet ${PROJECT_NAME})

#rosbuild_add_compile_flags(octomap_server_nodelet -msse4.2 -mpreferred-stack-boundary=4 -march=native)

rosbuild_add_executable(footprint_eraser_node
	src/footprint_eraser_node.cpp
	)
	
