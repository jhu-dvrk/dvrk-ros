cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()

# add dynamic reconfigure api for hokuyo
rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()


add_definitions(-fPIC)

rosbuild_add_boost_directories()


rosbuild_add_library(daVinciGazeboJointController src/daVinciGazeboJointController.cpp)
rosbuild_link_boost(daVinciGazeboJointController thread)


