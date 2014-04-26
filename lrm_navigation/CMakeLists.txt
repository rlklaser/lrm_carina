cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

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

#rosbuild_add_executable(obstacle_avoidance 
#	src/obstacle_avoidance.cpp
#	src/cVFH.cpp
#	src/cObstacleDetection.cpp
#	src/cObstacleAvoidance.cpp
#	src/cGPSNavigation.cpp
#	src/cFindPlane.cpp
#	)
	
rosbuild_add_library(lrm_navigation
	src/transform_cloud_nodelet.cpp
	)
rosbuild_add_boost_directories()

#find_package(VXL)
#if(VXL_FOUND)
#  include(${VXL_CMAKE_DIR}/UseVXL.cmake)
#endif(VXL_FOUND)
#
#rosbuild_add_executable(footprint_eraser
#	src/footprint_eraser.cpp
#	)
#target_link_libraries(footprint_eraser vnl vgl)


rosbuild_add_executable(send_goal
	src/send_goal.cpp
	)

