cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Release)

#include (FindPkgConfig)
#if (PKG_CONFIG_FOUND)
#  pkg_check_modules(GAZEBO gazebo)
#endif()

#find_package(gazebo REQUIRED)
#include_directories(${GAZEBO_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})

##add_library(hello_world SHARED hello_world.cc)
##target_link_libraries(hello_world ${GAZEBO_libraries})


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set (COMPILE_FLAGS -msse4.2 -mpreferred-stack-boundary=4 -march=native)
add_definitions(-Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)

rosbuild_add_boost_directories()

#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(carina_controller src/CarinaController.cpp)

rosbuild_add_library(suspension_controller src/SuspensionController.cpp)

rosbuild_add_library(encoders_controller src/EncodersController.cpp)

#add_definitions(-fPIC)
#rosbuild_add_library(LRMVehiclePlugin src/DRCVehiclePlugin.cc)

#rosbuild_add_library(ackermann_plugin src/ackermann_plugin.cpp)

#rosbuild_add_library(simBotBaseAckermannController src/simBotBaseAckermannController.cpp)
#rosbuild_link_boost(simBotBaseAckermannController thread)

rosbuild_add_library(AckermannControllerPlugin src/AckermannControllerPlugin.cpp)
rosbuild_link_boost(AckermannControllerPlugin thread)

rosbuild_add_library(carina_world_plugin src/CarinaWorldPlugin.cpp)

#rosbuild_add_library(projector_plugin src/projector.cc)

rosbuild_add_library(AckermannPlugin src/AckermannPlugin.cpp)
rosbuild_link_boost(AckermannPlugin thread)

rosbuild_add_library(lrm_gazebo_ros_controller_manager src/gazebo_ros_controller_manager.cpp)
rosbuild_link_boost(lrm_gazebo_ros_controller_manager thread)
#rosbuild_add_compile_flags(lrm_gazebo_ros_controller_manager $COMPILE_FLAGS)

rosbuild_add_library(lrm_gazebo_ros_prosilica src/gazebo_ros_prosilica.cpp)
rosbuild_link_boost(lrm_gazebo_ros_prosilica thread)
target_link_libraries(lrm_gazebo_ros_prosilica gazebo_ros_camera_utils DepthCameraPlugin)
#rosbuild_add_compile_flags(lrm_gazebo_ros_prosilica $COMPILE_FLAGS)

rosbuild_add_library(lrm_gazebo_ros_gps src/gazebo_ros_gps.cpp)
#rosbuild_add_compile_flags(lrm_gazebo_ros_gps $COMPILE_FLAGS)

rosbuild_add_library(lrm_gazebo_ros_imu src/gazebo_ros_imu.cpp)
rosbuild_link_boost(lrm_gazebo_ros_imu thread)
#rosbuild_add_compile_flags(lrm_gazebo_ros_imu $COMPILE_FLAGS)

rosbuild_add_library(lrm_gazebo_ros_magnetic src/gazebo_ros_magnetic.cpp)


