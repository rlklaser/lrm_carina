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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(roboteq_node 
	src/roboteq_node.cpp 
	src/roboteq.cpp 
	src/serial.cpp 
	src/AX2550.cpp 
	src/HDC2450.cpp)
	
rosbuild_add_executable(can_node 
	src/cCAN.cpp 
	src/can_node.cpp)
	
rosbuild_add_executable(throttle_node 
	src/throttle_node.cpp 
	src/throttle.cpp 
	src/serial.cpp 
	src/vcmdas1_driver.cpp 
	src/arduino_throttle.cpp 
	src/vcmdas1_throttle.cpp)
	
rosbuild_add_executable(teleoperation_by_joy 
	src/teleoperation_by_joy.cpp)
	
rosbuild_add_executable(nav_message_node 
	src/nav_message_node.cpp)

rosbuild_add_executable(car_message_node 
	src/car_message_node.cpp)
	
#rosbuild_add_executable(cruise_control 
#	src/ControlVelocityBrake.c 
#	src/cruise_control.cpp)
		