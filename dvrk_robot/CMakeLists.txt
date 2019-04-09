#
# (C) Copyright 2015-2018 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.1)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

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
     cisstCommonQt
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

find_package (cisst 1.0.11 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  include (${CISST_USE_FILE})

  # include_directories(include)
  include_directories (${catkin_INCLUDE_DIRS})

  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

  catkin_package (INCLUDE_DIRS include "${CATKIN_DEVEL_PREFIX}/include"
                  LIBRARIES dvrk_utilities
                  CATKIN_DEPENDS cisst_msgs cisst_ros_bridge geometry_msgs roscpp std_msgs)


  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  find_package (sawRobotIO1394          1.7.0 REQUIRED)
  find_package (sawControllers          1.7.0 REQUIRED)
  find_package (sawIntuitiveResearchKit 1.7.0 REQUIRED)

  include_directories (
    ${dvrk_robot_SOURCE_DIR}/include
    ${CATKIN_DEVEL_PREFIX}/include
    ${sawRobotIO1394_INCLUDE_DIR}
    ${sawControllers_INCLUDE_DIR}
    ${sawIntuitiveResearchKit_INCLUDE_DIR}
    )

  link_directories (
    ${sawRobotIO1394_LIBRARY_DIR}
    ${sawControllers_LIBRARY_DIR}
    ${sawIntuitiveResearchKit_LIBRARY_DIR}
    )

  cisst_data_generator (dvrk_utilities
                        "${CATKIN_DEVEL_PREFIX}/include" # where to save the file
                        "dvrk_utilities/"    # sub directory for include
                        src/dvrk_topics_version.cdg)

  add_library (dvrk_utilities
               include/dvrk_utilities/dvrk_add_topics_functions.h
               src/dvrk_add_topics_functions.cpp
               include/dvrk_utilities/dvrk_console.h
               src/dvrk_console.cpp
               ${dvrk_utilities_CISST_DG_SRCS}
               ${dvrk_utilities_CISST_DG_HDRS})

    target_link_libraries (
      dvrk_utilities
      ${sawRobotIO1394_LIBRARIES}
      ${sawIntuitiveResearchKit_LIBRARIES}
      ${sawControllers_LIBRARIES}
      ${catkin_LIBRARIES}
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
      ${catkin_LIBRARIES}
    )
    cisst_target_link_libraries (${_executable} ${REQUIRED_CISST_LIBRARIES})
    target_link_libraries (${_executable} dvrk_utilities)
  endforeach (_executable)

endif (cisst_FOUND_AS_REQUIRED)
