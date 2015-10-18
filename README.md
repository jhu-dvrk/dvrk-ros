da Vinci Research Kit ROS
====================
This repository has code related to daVinci Research Kit (dVRK) ROS packages.
See https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki

# Install
We use the catkin build tools, NOT catkin_make.  Please don't use catkin_make
* Download and compile the cisst libraries and SAW components for the dVRK, see the dVRK tutorial wiki: https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros
* Download and compile dvrk-ros: https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild

# How to run the code
* **simulation:** see dvrk_teleop/README.md for details
* **robot:** see dvrk_robot/README.md for details 

# List of Branches
* master: current stable version
* dev\_groovy: groovy development verison
* catkin: catkin version 

# List of Packages:
* dv\_gazebo\_plugins: from WPI Nirav, don't know how to use Nirav? 
* dvrk_kinematics:
  * kinematics model of MTM/PSM
* dvrk_model:
  * MTM + PSM CAD models & meshes
  * rviz configs
  * launch files
* dvrk_teleop:
  * dummy teleop component with a Qt GUI
* dvrk_robot: 
  * main file to start robot, publish & subscribe ros topics 
