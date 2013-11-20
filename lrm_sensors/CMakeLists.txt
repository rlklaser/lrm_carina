cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(sensors_proc
	#src/nodelets/split_bumblebee.cpp
	src/nodelets/split_bb2_nodelet.cpp
	src/nodelets/split_xb3_nodelet.cpp
	src/nodelets/split_ladybug_nodelet.cpp
	)
	
#rosbuild_add_library(cloud_plane_reproject_node
#	src/cFindPlane.cpp
#	src/cloud_plane_reproject_node.cpp
#	)


#add_definitions(-std=c++11)

rosbuild_add_executable(fake_clock_node 
    src/fake_clock_node.cpp)
    
rosbuild_add_executable(cloud_to_scan 
    src/cloud_to_scan.cpp)
    
#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_library(pointcloud_to_scan 
    src/pointcloud_to_scan.cpp 
    src/cloud_throttle.cpp)
