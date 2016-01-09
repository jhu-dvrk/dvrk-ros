#
# (C) Copyright 2015-2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8.3)
project (dvrk_robot)

## find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              cisst_msgs
              cisst_ros_bridge
              geometry_msgs
              roscpp
              std_msgs
              roslib
              )

# find cisst and make sure the required libraries have been compiled
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

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  include (${CISST_USE_FILE})


  # include_directories(include)
  include_directories (${catkin_INCLUDE_DIRS})

  catkin_package (INCLUDE_DIRS include
                  LIBRARIES dvrk_utilities
                  CATKIN_DEPENDS cisst_msgs cisst_ros_bridge geometry_msgs roscpp std_msgs
                  DEPENDS ${REQUIRED_CISST_LIBRARIES})

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  cisst_find_saw_component (sawRobotIO1394          1.3.0 REQUIRED)
  cisst_find_saw_component (sawIntuitiveResearchKit 1.3.0 REQUIRED)
  cisst_find_saw_component (sawControllers          1.3.0 REQUIRED)
  cisst_find_saw_component (sawConstraintController REQUIRED)

  include_directories (
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${sawRobotIO1394_INCLUDE_DIR}
    ${sawControllers_INCLUDE_DIR}
    ${sawConstraintController_INCLUDE_DIR}
    ${sawIntuitiveResearchKit_INCLUDE_DIR}
    )

  link_directories (
    ${sawRobotIO1394_LIBRARY_DIR}
    ${sawControllers_LIBRARY_DIR}
    ${sawConstraintController_LIBRARY_DIR}
    ${sawIntuitiveResearchKit_LIBRARY_DIR}
    )

  add_library (dvrk_utilities
               include/dvrk_utilities/dvrk_add_topics_functions.h
               src/dvrk_add_topics_functions.cpp
               include/dvrk_utilities/dvrk_console.h
               src/dvrk_console.cpp)
    target_link_libraries (
      dvrk_utilities
      ${sawRobotIO1394_LIBRARIES}
      ${sawIntuitiveResearchKit_LIBRARIES}
      ${sawControllers_LIBRARIES}
      ${sawConstraintController_LIBRARIES}
    )
    cisst_target_link_libraries (dvrk_utilities ${REQUIRED_CISST_LIBRARIES})

  set (_EXECUTABLES dvrk_mtm_ros dvrk_psm_ros dvrk_ecm_ros dvrk_full_ros dvrk_console_json)
  foreach (_executable ${_EXECUTABLES})
    add_executable (${_executable} src/${_executable}.cpp)
    target_link_libraries (
      ${_executable}
      ${sawRobotIO1394_LIBRARIES}
      ${sawIntuitiveResearchKit_LIBRARIES}
      ${sawControllers_LIBRARIES}
      ${sawConstraintController_LIBRARIES}
      ${catkin_LIBRARIES}
    )
    cisst_target_link_libraries (${_executable} ${REQUIRED_CISST_LIBRARIES})
    target_link_libraries (${_executable} dvrk_utilities)
  endforeach (_executable)

endif (cisst_FOUND_AS_REQUIRED)
