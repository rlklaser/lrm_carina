cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

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

rosbuild_add_executable(optflow src/ic2020_optflow.cpp)


#rosbuild_add_library(lrm_wrappers 
#	src/disparity.cpp 
#	)

rosbuild_add_executable(retinal_node src/retinal_node.cpp)
#target_link_libraries(retinal_node ${OpenCV_LIBS})

	
rosbuild_add_executable(cloud_cut src/cloud_cut.cpp)
#link_libraries(cloud_cut 
#	#${PCL_COMMON_LIBRARIES}
#	)
	

#rosbuild_add_executable(keyboard src/keyboard.cpp)


rosbuild_add_executable(plane_segment src/plane_segment.cpp)

rosbuild_add_executable(surface_reconstruction src/surface_reconstruction.cpp)

	