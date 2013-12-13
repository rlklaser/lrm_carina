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

add_definitions(-std=c++0x)
add_definitions(-msse3 -msse4.1 -msse4.2)
add_definitions(-mfpmath=sse -Ofast -flto -march=native)

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

rosbuild_add_library(carina_controller src/carina_controller.cpp)

rosbuild_add_executable(pose_velocity_node 
	src/pose_velocity_node.cpp)
	
rosbuild_add_executable(teleop_keyboard 
	src/teleop_keyboard.cpp)
	
rosbuild_add_library(steering_controller src/steering_controller.cpp)

rosbuild_add_executable(cmd_vel_to_messages 
	src/cmd_vel_to_messages.cpp)

rosbuild_add_executable(teleoperation_by_joy 
	src/teleoperation_by_joy.cpp)
	
rosbuild_add_executable(robot_state_publisher 
    src/robot_state/robot_state_publisher.cpp 
    src/robot_state/treefksolverposfull_recursive.cpp
    src/robot_state/joint_state_listener.cpp )
		