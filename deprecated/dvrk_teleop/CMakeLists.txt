cmake_minimum_required(VERSION 2.8.3)
project(dvrk_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cisst_ros_bridge
  dvrk_kinematics
  geometry_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)

set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonXML
  cisstVector
  cisstVectorQt
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstRobot
  cisstNumerical
  cisstQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dvrk_teleop
#  CATKIN_DEPENDS cisst_ros_bridge geometry_msgs roscpp rospy sensor_msgs std_msgs tf tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


# dvrk_teleop
add_executable(dvrk_teleop
  src/teleop.cpp
  include/dvrk_teleop/mtsTeleop.h
  src/mtsTeleop.cpp)
target_link_libraries (dvrk_teleop ${catkin_LIBRARIES})
cisst_target_link_libraries (dvrk_teleop ${REQUIRED_CISST_LIBRARIES})


# dvrk_teleop_gui
if (CISST_HAS_QT4)
  qt4_wrap_cpp (QT_WRAPPED_CPP include/dvrk_teleop/dvrkTeleopQWidget.h)
else ()
  set (CMAKE_AUTOMOC ON)
endif ()

add_executable(dvrk_teleop_gui
  include/dvrk_teleop/dvrkTeleopQWidget.h
  src/dvrkTeleopQWidget.cpp
  src/teleopGUI.cpp
  ${QT_WRAPPED_CPP})

target_link_libraries (dvrk_teleop_gui ${catkin_LIBRARIES})
cisst_target_link_libraries (dvrk_teleop_gui ${REQUIRED_CISST_LIBRARIES})

if (CISST_HAS_QT5)
  qt5_use_modules (dvrk_teleop_gui Core Widgets OpenGL)
endif (CISST_HAS_QT5)
