cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(voxel_grid src/voxel_grid.cpp)

rosbuild_add_executable(voxel_grid_node 
	src/voxel_grid_main.cpp)
target_link_libraries(voxel_grid_node voxel_grid)

rosbuild_add_gtest(test/voxel_grid_tests test/voxel_grid_tests.cpp)
target_link_libraries(test/voxel_grid_tests voxel_grid)
