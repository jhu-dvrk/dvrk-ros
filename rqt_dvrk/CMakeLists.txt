cmake_minimum_required(VERSION 2.8.3)
project(rqt_dvrk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

# catkin python
catkin_python_setup()

# catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rqt_dvrk
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

