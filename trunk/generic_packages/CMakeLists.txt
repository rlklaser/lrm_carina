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

#find_package(OpenCV REQUIRED)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(stitch src/stitching_detailed.cpp)
#target_link_libraries(stitch ${OpenCV_LIBS})

#rosbuild_add_executable(retina src/retinaDemo.cpp)
#target_link_libraries(retina ${OpenCV_LIBS})


#find_package(PCL 1.2 REQUIRED)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


#rosbuild_add_executable(viz_image_node src/viz_image_node.cpp)
#target_link_libraries(viz_image_node 
#	${OpenCV_LIBS}
#	${PCL_LIBRARIES}
#	)

#rosbuild_add_compile_flags(viz_image_node -msse4.2 -mavx -mpreferred-stack-boundary=4 -march=native)


#find_package(ns-3 3.10 REQUIRED)

rosbuild_add_executable(wifi_sim 
	src/wifi_sim.cpp)

#rosbuild_add_compile_flags(wifi_sim 
#	-lns3-core 
#	-lns3-mobility 
#	-lns3-network 
#	-lns3-spectrum 
#	-lns3-propagation 
#	-lns3-wifi
#	-lsn3-aodv
#	-lsn3-applications
#	-lsn3-bridge
#	-lsn3-config-store
#	-lsn3-csma
#	-lsn3-csma-layout
#	-lsn3-dsdv
#	-lsn3-emu
#	-lsn3-energy
#	-lsn3-flow-monitor
#	-lsn3-internet
#	-lsn3-lte
#	-lsn3-mesh
#	-lsn3-mpi
#	-lsn3-netanim
#	-lsn3-nix-verctor-routing
#	-lsn3-olsr
#	-lsn3-point-to-point
#	-lsn3-stats
#	-lsn3-tap-bridge
#	-lsn3-test
#	-lsn3-tools
#	-lsn3-topology-read
#	-lsn3-uan
#	-lsn3-virtual-net-device
#	-lsn3-visualizer
#	-lsn3-wimax
#	)

#link_directories(/usr/lib)

#target_link_libraries(wifi_sim
#	ns3-core 
#	ns3-mobility 
#	ns3-network 
#	ns3-spectrum 
#	ns3-propagation 
#	ns3-wifi
#	sn3-aodv
#	sn3-applications
#	sn3-bridge
#	sn3-config-store
#	sn3-csma
#	sn3-csma-layout
#	sn3-dsdv
#	sn3-emu
#	sn3-energy
#	sn3-flow-monitor
#	sn3-internet
#	sn3-lte
#	sn3-mesh
#	sn3-mpi
#	sn3-netanim
#	sn3-nix-verctor-routing
#	sn3-olsr
#	sn3-point-to-point
#	sn3-stats
#	sn3-tap-bridge
#	sn3-test
#	sn3-tools
#	sn3-topology-read
#	sn3-uan
#	sn3-virtual-net-device
#	sn3-visualizer
#	sn3-wimax
#	)

rosbuild_add_executable(sba_system_setup src/sba_system_setup.cpp)

rosbuild_add_executable(sba_from_file src/sba_from_file.cpp)

rosbuild_add_boost_directories()

rosbuild_add_executable(two_gps_log src/two_gps_log.cpp)
rosbuild_link_boost(two_gps_log signals)
