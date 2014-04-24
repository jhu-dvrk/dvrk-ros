cmake_minimum_required(VERSION 2.8.3)
project(sim_mtm)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(sim_mtm_node src/sim_mtm_pose_correction.cpp)

 target_link_libraries(sim_mtm_node
   ${catkin_LIBRARIES} )

