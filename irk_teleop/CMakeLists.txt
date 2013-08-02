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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


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

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # sawRobotIO1394 has been compiled within cisst, we should find it automatically
  cisst_find_saw_component (sawRobotIO1394 REQUIRED)
  cisst_find_saw_component (sawIntuitiveResearchKit REQUIRED)
  cisst_find_saw_component (sawControllers REQUIRED)

  if (sawRobotIO1394_FOUND)

    include_directories (${sawRobotIO1394_INCLUDE_DIR}
      ${sawIntuitiveResearchKit_INCLUDE_DIR}
      ${sawControllers_INCLUDE_DIR})
    
    link_directories (${sawRobotIO1394_LIBRARY_DIR}
      ${sawIntuitiveResearchKit_LIBRARY_DIR}
      ${sawControllers_LIBRARY_DIR})
    
    rosbuild_add_executable(irk_teleop
      src/teleop.cpp
      include/irk_teleop/mtsTeleop.h
      src/mtsTeleop.cpp)
    
    # link against non cisst libraries and cisst components
    target_link_libraries (irk_teleop
      ${sawRobotIO1394_LIBRARIES}
      ${sawIntuitiveResearchKit_LIBRARIES}
      ${sawControllers_LIBRARIES})
    
    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (irk_teleop ${REQUIRED_CISST_LIBRARIES})
    
    
    qt4_wrap_cpp (QT_WRAPPED_CPP include/irk_teleop/irkTeleopQWidget.h)
    rosbuild_add_executable(irk_teleop_gui
      include/irk_teleop/irkTeleopQWidget.h
      src/irkTeleopQWidget.cpp
      src/teleopGUI.cpp
      ${QT_WRAPPED_CPP})
    
    # link against non cisst libraries and cisst components
    target_link_libraries (irk_teleop_gui
      ${sawRobotIO1394_LIBRARIES}
      ${sawIntuitiveResearchKit_LIBRARIES}
      ${sawControllers_LIBRARIES})
    
    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (irk_teleop_gui ${REQUIRED_CISST_LIBRARIES})
    
  endif (sawRobotIO1394_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
