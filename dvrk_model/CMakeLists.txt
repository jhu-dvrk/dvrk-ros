cmake_minimum_required(VERSION 2.8.3)
project(dvrk_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  )

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES cisst_ros_bridge
  # CATKIN_DEPENDS message_runtime
  # DEPENDS system_lib
)
