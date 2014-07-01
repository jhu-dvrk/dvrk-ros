cmake_minimum_required(VERSION 2.8.3)
project(dvrk_robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cisst_msgs
  cisst_ros_bridge
  geometry_msgs
  roscpp
  std_msgs
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
  cisstParameterTypesQt
  cisstRobot
  cisstNumerical
  cisstQt
  )



# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dvrk_robot
#  CATKIN_DEPENDS cisst_msgs cisst_ros_bridge geometry_msgs
#                 roscpp std_msgs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (cisst_FOUND_AS_REQUIRED)

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  cisst_find_saw_component (sawRobotIO1394 REQUIRED)
  cisst_find_saw_component (sawIntuitiveResearchKit REQUIRED)
  cisst_find_saw_component (sawControllers REQUIRED)

  message(STATUS "sawControllers: " ${sawControllers_LIBRARY_DIR})
  message(STATUS "sawControllers_LIBRARIES: " ${sawControllers_LIBRARIES})

  include_directories (
    ${sawRobotIO1394_INCLUDE_DIR}  
    ${sawControllers_INCLUDE_DIR}
    ${sawIntuitiveResearchKit_INCLUDE_DIR}
    )

  link_directories (
    ${sawRobotIO1394_LIBRARY_DIR}
    ${sawControllers_LIBRARY_DIR}
    ${sawIntuitiveResearchKit_LIBRARY_DIR}
    )

  set(ROBOTSRC dvrk_mtm_ros dvrk_psm_ros dvrk_full_ros)
  foreach(_robot ${ROBOTSRC})
    add_executable (${_robot} src/${_robot}.cpp)
    target_link_libraries (${_robot}
      ${sawRobotIO1394_LIBRARIES}
      ${sawIntuitiveResearchKit_LIBRARIES}
      ${sawControllers_LIBRARIES}
      ${catkin_LIBRARIES}
    )
    cisst_target_link_libraries (${_robot} ${REQUIRED_CISST_LIBRARIES})

    if (CISST_HAS_QT5)
      qt5_use_modules (${_robot} Core Widgets OpenGL)
    endif (CISST_HAS_QT5)

  endforeach(_robot)

endif(cisst_FOUND_AS_REQUIRED)


