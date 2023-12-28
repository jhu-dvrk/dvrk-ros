cmake_minimum_required (VERSION 3.10)
project (dvrk_hrsv_widget VERSION 2.2.1)

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  )

# find Qt
find_package(Qt5Widgets)
find_package(Qt5Core REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories (
  ${catkin_INCLUDE_DIRS}
  ${dvrk_hrsv_widget_SOURCE_DIR}/include
  ${dvrk_hrsv_widget_BINARY_DIR}/include
)

catkin_package ()

add_executable (hrsv_widget
  include/hrsv_widget.h
  src/hrsv_widget.cpp
  src/main.cpp)

target_link_libraries (
  hrsv_widget
  Qt5::Widgets
  ${catkin_LIBRARIES})
