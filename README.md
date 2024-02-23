da Vinci Research Kit ROS
====================
This repository has code related to daVinci Research Kit (dVRK) ROS packages.
See https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki

# Install

* ROS 1: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild
* ROS 2: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/BuildROS2
  
# List of Packages:
* `dvrk_robot` **[maintained]** 
  * Main file to start dVRK (`dvrk_console_json`), publish & subscribe ros topics
  * Launch files to start RViz with geometric simulation
* `dvrk_model` **[mostly maintained]**
  * CAD models & meshes
  * RViz configs
  * Launch files
* `dvrk_python` **[maintained]**
  * Python classes using ROS topics
  * Simple API to control any dVRK arm as well as console and teleoperation components
* `dvrk_matlab` **[maintained, missing some features]**
  * Matlab classes using ROS topics
  * Requires Robotics Toolkit (2015a and above), any OS (Windows, Mac, Linux)
  * Simple API to control any dVRK arm
  * No support for console (yet)
* `dvrk_hrsv_widget` **[maintained]**
  * Qt based application that launches two simple widgets to be displayed in stereo console
  * Provides simple text based feedback to operator to understand current state of dVRK components
* `dvrk_arms_from_ros` **[maintained]**
  * cisst/SAW plugin for the dVRK C++ stack to use any arm with a dVRK compatible set of ROS topics
  * This plugin allows to use ROS as middleware between two dVRK sets of arms on different PCs (distant tele-operation)



