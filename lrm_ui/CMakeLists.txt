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


set(QT_USE_QTSVG TRUE)
set(QT_USE_QTXML TRUE)
set(QT_USE_QTSCRIPT TRUE)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#rosbuild_include(qt_build qt-ros)
#rosbuild_prepare_qt4(QtCore QtGui)

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS 
#    include/ihm/*.hpp 
#    include/analogwidgets/*.h 
#    include/analogwidgets/chart/*.h 
#    include/analogwidgets/chart/standard/*.h)

#QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

#INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})

#IF(NOT COMPILE_ANALOG_LIB)
#   EXEC_PROGRAM(${PROJECT_SOURCE_DIR}/.create_analog_lib.sh )
#   SET(COMPILE_ANALOG_LIB TRUE)
#ENDIF(NOT GTK_PKG_FLAGS)

#LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/AnalogWidgets/analogwidgets)

#rosbuild_add_executable(ihm 
#    ${QT_SOURCES} 
#    ${QT_RESOURCES_CPP} 
#    ${QT_FORMS_HPP} 
#    ${QT_MOC_HPP})
    
#target_link_libraries(ihm 
#    ${QT_LIBRARIES} 
#    -lanalogwidgets 
#    -lQtSvg 
#    -lQtScript)
