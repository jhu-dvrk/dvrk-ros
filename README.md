da Vinci Research Kit ROS
====================
This repository has code related to daVinci Research Kit (dVRK) ROS packages.
See https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki

# Install
We use the catkin build tools, NOT catkin_make.  Please don't use catkin_make
* Download and compile the cisst libraries and SAW components for the dVRK, see the dVRK tutorial wiki: https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros
* Download and compile dvrk-ros: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild

# List of Packages:
* dvrk_robot **[maintained]** 
  * Main file to start dVRK (`dvrk_console_json`), publish & subscribe ros topics
  * Launch files to start RViz with geometric simulation
* dvrk_model **[maintained]**
  * CAD models & meshes
  * RViz configs
  * Launch files
* dvrk_python **[maintained]**
  * Python classes using ROS topics
  * Simple API to control any dVRK arm as well as console and teleoperation components
* dvrk_matlab **[maintained, missing some features]**
  * Matlab classes using ROS topics
  * Requires Robotics Toolkit (2015a and above), any OS (Windows, Mac, Linux)
  * Simple API to control any dVRK arm
  * No support for console (yet)
* dv\_gazebo\_plugins **[not maintained]**
  * from WPI Nirav, don't know how to use Nirav? 
* dvrk_kinematics  **[not maintained]**
  * kinematics model of MTM/PSM
* dvrk_teleop  **[not maintained]**
  * dummy teleop component with a Qt GUI


