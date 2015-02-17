#
# Author(s):  Anton Deguet
# Created on: 2015-02-16
#
# (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8.3)
project (dvrk_arm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              cisst_msgs
              cisst_ros_bridge
              dvrk_bridges
              geometry_msgs
              roscpp
              std_msgs
              roslaunch
              )

# catkin magic
catkin_package ()

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

if (cisst_FOUND_AS_REQUIRED)
  include (${CISST_USE_FILE})

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  cisst_find_saw_component (sawRobotIO1394 REQUIRED)
  cisst_find_saw_component (sawIntuitiveResearchKit REQUIRED)
  cisst_find_saw_component (sawControllers REQUIRED)
  cisst_find_saw_component (sawConstraintController REQUIRED)

  include_directories (${catkin_INCLUDE_DIRS}
                       ${sawRobotIO1394_INCLUDE_DIR}
                       ${sawControllers_INCLUDE_DIR}
                       ${sawConstraintController_INCLUDE_DIR}
                       ${sawIntuitiveResearchKit_INCLUDE_DIR}
                       )

  link_directories (${sawRobotIO1394_LIBRARY_DIR}
                    ${sawControllers_LIBRARY_DIR}
                    ${sawConstraintController_LIBRARY_DIR}
                    ${sawIntuitiveResearchKit_LIBRARY_DIR}
                    )

  add_executable (dvrk_arm src/dvrk_arm.cpp)
  target_link_libraries (dvrk_arm
                         ${sawRobotIO1394_LIBRARIES}
                         ${sawIntuitiveResearchKit_LIBRARIES}
                         ${sawControllers_LIBRARIES}
                         ${sawConstraintController_LIBRARIES}
                         ${catkin_LIBRARIES}
                         )
  cisst_target_link_libraries (dvrk_arm ${REQUIRED_CISST_LIBRARIES})

  # check simple errors in scripts
  roslaunch_add_file_check(launch)

endif (cisst_FOUND_AS_REQUIRED)


