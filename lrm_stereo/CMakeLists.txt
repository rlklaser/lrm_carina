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
#set(ROS_BUILD_TYPE Debug)

#add_definitions(-std=c++0x)
add_definitions(-msse3 -msse4.1 -msse4.2)
add_definitions(-mfpmath=sse -Ofast -flto -march=native)

add_definitions(-DEIGEN_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)


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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(lrm_stereo 
	src/disparity.cpp 
	src/disparity_bm.cpp 
	#src/disparity_bm_cuda.cpp 
	#src/disparity_bm_ocl.cpp 
	
	src/feature_matcher.cpp
	src/ros_pcl_stereo.cpp
	src/point_cloud_colorless.cpp
	src/point_cloud2.cpp
	src/segmentation_nodelet.cpp
	src/moving_roi_nodelet.cpp

	src/throttle_stereo.cpp
	src/throttle_stereo_nodelet.cpp
		
	src/region_growing.cpp
	src/region_growing_nodelet.cpp
	
	src/flatten_ground.cpp
	src/flatten_ground_nodelet.cpp

	src/zigzag_cut.cpp
	src/zigzag_cut_nodelet.cpp
	)
rosbuild_add_boost_directories()
#target_link_libraries(lrm_stereo pcl_ros_filters)

rosbuild_add_executable(fake_cloud src/fake_cloud.cpp)
rosbuild_add_openmp_flags(fake_cloud)

#rosbuild_add_library(lrm_stereo 
#	src/disparity_bm_cuda.cpp 
#	)

#rosbuild_add_executable(throttle_stereo 
#	src/throttle_stereo.cpp 
#	)

rosbuild_add_executable(throttle_stereo_node 
    src/throttle_stereo.cpp 
	src/throttle_stereo_node.cpp 
	)
