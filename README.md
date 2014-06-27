da Vinci Research Kit ROS
====================
This repository has code related to daVinci\_research\_kit ROS packages. 

# Install 
* download & build cisst, see dvrk tutorial wiki for detail 
* download & compile cisst-ros

```sh
# cd to catkin ws src dir
cd /PATH/TO/CATKIN_WS/src
# clone repo
git clone http://github.com/jhu-cisst/cisst-ros.git

# build 
cd ..
catkin_make
```
* download & compile dvrk-ros

```sh
# cd to catkin ws src dir
cd /PATH/TO/CATKIN_WS/src
# clone repo
git clone http://github.com/jhu-dvrk/dvrk-ros.git
# build 
cd ..
catkin_make
```

# How to run the code
* **simulation:** see dvrk_teleop/README.md for details
* **robot:** see dvrk_robot/README.md for details 

# List of Branches
* master: current stable version
* dev\_groovy: groovy development verison
* catkin: catkin version 

# List of Packages:
* dv\_gazebo\_plugins: from WPI Nirav, don't know how to use Nirav? 
* dvrk\_joint\_publisher: publishes joint state
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
* dvrk\_joint\_publisher:
  * publish named joint states (position + velocity)
