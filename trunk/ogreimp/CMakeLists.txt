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

rosbuild_add_boost_directories()

#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


#find_package(ASSIMP QUIET)
#if (NOT ASSIMP_FOUND)
#  pkg_check_modules(ASSIMP assimp)
#endif()
#if (ASSIMP_FOUND)
#  if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
#    set(IS_ASSIMP3 1)
#    add_definitions(-DIS_ASSIMP3)
#    message(STATUS "Found assimp v3")
#  else()
#    message(STATUS "Found assimp v2")
#  endif()
#  include_directories(${ASSIMP_INCLUDE_DIRS})
#  link_directories(${ASSIMP_LIBRARY_DIRS})
#else()
#  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assimping assimp v2")
#  set(ASSIMP_LIBRARIES assimp)
#  set(ASSIMP_LIBRARY_DIRS)
#  set(ASSIMP_CXX_FLAGS)
#  set(ASSIMP_CFLAGS_OTHER)
#  set(ASSIMP_LINK_FLAGS)
#  set(ASSIMP_INCLUDE_DIRS)
#  set(IS_ASSIMP3 0) # most likely not
#endif()

rosbuild_add_executable(ogreimp src/ogreimp.cpp)
rosbuild_link_boost(ogreimp filesystem)
