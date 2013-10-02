cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# auto-generated dynamic reconfiguration GUI
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_executable(
	split_node 
	src/split_node.cpp
	)

rosbuild_add_executable(
	split_xb3_node 
	src/split_xb3_node.cpp
	)
	
		
#rosbuild_add_compile_flags(split_node -msse4.2 -mpreferred-stack-boundary=4 -march=native)

set(DRIVER_SOURCES 
  src/nodes/driver1394stereo.cpp
  src/nodes/dev_camera1394stereo.cpp
  src/nodes/featuresstereo.cpp
  src/nodes/format7stereo.cpp
  src/nodes/modes.cpp)
  
set(DC1394 dc1394)
  
#find_package(DC1394 REQUIRED)

rosbuild_add_executable(camera1394stereo_node 
	src/nodes/camera1394stereo_node.cpp 
	${DRIVER_SOURCES})
target_link_libraries(camera1394stereo_node ${DC1394})

rosbuild_add_library(camera1394stereo_nodelet 
	src/nodes/nodelet.cpp 
	${DRIVER_SOURCES})
target_link_libraries(camera1394stereo_nodelet ${DC1394})

