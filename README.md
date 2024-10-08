# :warning:

This repository is now archived and used for older dVRK distributions.  As of August 2024, the packages listed below have been moved to different repositories.  The new repositories contains code for both ROS1 and ROS2.

* `dvrk_robot`, `dvrk_hrsv_widget` and `dvrk_arms_from_ros` are now in https://github.com/jhu-dvrk/sawIntuitiveResearchKit under `ros`
* Video launch files that used to be under `dvrk_robot` are now in https://github.com/jhu-dvrk/dvrk_video
* `dvrk_model` has been split into 2 repositories:
  *  https://github.com/jhu-dvrk/dvrk_model for launch files, urdf, rviz and meshes
  *  https://github.com/jhu-dvrk/dvrk_cad for CAD files (larger files)
* `dvrk_python` is now in https://github.com/jhu-dvrk/dvrk_python
* `dvrk_matlab` has not been migrated yet

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



