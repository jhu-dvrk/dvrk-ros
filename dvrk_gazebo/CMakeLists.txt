cmake_minimum_required(VERSION 2.8.3)
project(dvrk_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  std_msgs
  sensor_msgs
)

find_package(
gazebo REQUIRED)


catkin_package(
  DEPENDS
  gazebo_ros
  CATKIN_DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

 add_library(dv_adnan_controller include/dvrk_gazebo_controller.h
   src/dvrk_gazebo_controller.cpp
 )
 target_link_libraries(dv_adnan_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

