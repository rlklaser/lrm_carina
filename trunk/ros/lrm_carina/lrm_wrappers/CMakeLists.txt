cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)
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

#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})

#find_package(OpenCV REQUIRED)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


add_definitions(-Wno-deprecated -DEIGEN_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
add_definitions(-msse3 -msse4.1 -msse4.2 -mfpmath=sse)
#add_definitions(-mavx -mfpmath=sse)

#rosbuild_add_executable(optflow src/ic2020_optflow.cpp)

#rosbuild_add_library(lrm_wrappers 
	#src/disparity.cpp
	#src/bilateralz.cpp
	#include/lrm_wrappers/impl/bilateralz.hpp
#	)

rosbuild_add_executable(retinal_node src/retinal_node.cpp)
#target_link_libraries(retinal_node ${OpenCV_LIBS})

	
rosbuild_add_executable(cloud_cut src/cloud_cut.cpp)
#link_libraries(cloud_cut 
#	#${PCL_COMMON_LIBRARIES}
#	)
	
#rosbuild_add_executable(keyboard src/keyboard.cpp)

rosbuild_add_executable(plane_segment src/plane_segment.cpp)

#rosbuild_add_executable(surface_reconstruction src/surface_reconstruction.cpp)

#rosbuild_add_executable(pointcloud_minmax_3d src/pointcloud_minmax_3d.cpp)

#rosbuild_add_executable(transform_pointcloud src/transform_pointcloud.cpp)

#rosbuild_add_executable(bilateral_filter_node src/bilateral_filter_node.cpp)

rosbuild_add_executable(cloud_sum_node src/cloud_sum.cpp)

#rosbuild_add_executable(cloud_register_node src/cloud_register.cpp)

#rosbuild_add_executable(cloud_don_segmentation src/cloud_don_segmentation.cpp)

##nodelets on lrm_stereo
##rosbuild_add_executable(region_growing 
##    src/region_growing.cpp)
##
##rosbuild_add_executable(region_growing_rgb 
##    src/region_growing_rgb.cpp)

rosbuild_add_executable(cluster_filter_node src/cluster_filter_node.cpp)

#rosbuild_add_executable(median_filter_node src/median_filter_node.cpp)

rosbuild_add_executable(principal_curvature_node 
    src/principal_curvature_node.cpp)

#rosbuild_add_executable(zigzag_filter_node 
#    src/zigzag_filter.cpp)

rosbuild_add_executable(moving_leastsquares_node 
    src/moving_leastsquares_node.cpp)
        
rosbuild_add_executable(smooth_axis_filter_node 
    src/smooth_axis_filter.cpp)

#rosbuild_add_executable(flatten_ground_filter_node 
#    src/flatten_ground_filter_node.cpp)    
    
rosbuild_add_executable(interactive_tf 
    src/interactive_tf.cpp)
