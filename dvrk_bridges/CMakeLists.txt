cmake_minimum_required (VERSION 2.8.3)

project (dvrk_bridges)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin
              REQUIRED COMPONENTS
              cisst_msgs
              cisst_ros_bridge
              geometry_msgs
              roscpp
              std_msgs
              )

# include_directories(include)
include_directories (${catkin_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include")

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes
     )

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})
include (${CISST_USE_FILE})

if (cisst_FOUND_AS_REQUIRED)

  catkin_package (INCLUDE_DIRS include
                  LIBRARIES dvrk_ros_bridges
                  CATKIN_DEPENDS cisst_msgs cisst_ros_bridge geometry_msgs roscpp std_msgs
                  DEPENDS ${REQUIRED_CISST_LIBRARIES})

  add_library (dvrk_ros_bridges src/dvrk_arm_bridge.cpp)
  cisst_target_link_libraries (dvrk_ros_bridges ${REQUIRED_CISST_LIBRARIES})

endif(cisst_FOUND_AS_REQUIRED)
