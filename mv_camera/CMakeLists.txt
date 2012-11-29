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

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_gensrv()

# auto-generated dynamic reconfiguration GUI
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

add_definitions(-DGENICAM_CACHE_V2_3='"${PROJECT_SOURCE_DIR}/genicam_cache"')


# common source files for both node and nodelet driver interfaces
set(DRIVER_SOURCES 
  src/driver_mv_camera.cpp
  src/dev_mv_camera.cpp
  src/features.cpp
  src/formats.cpp
  )

rosbuild_add_library(mv_camera_driver ${DRIVER_SOURCES})

rosbuild_add_executable(mv_camera_node src/mv_camera_node.cpp )
target_link_libraries(mv_camera_node  mv_camera_driver)

rosbuild_add_library(mv_camera_nodelet src/mv_camera_nodelet.cpp)
target_link_libraries(mv_camera_nodelet mv_camera_driver)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_add_executable(driverInterface src/driverInterface)
