daVinci Research Kit ROS
====================
This repository has code realted to daVinci\_research\_kit ROS packages. 

# Todo
* move to github 

# Install 
* download & build cisst, see dvrk tutorial wiki for detail 
* download & compile cisst\_ros\_integration

```
# cd to catkin ws src dir
cd /PATH/TO/CATKIN_WS/src
# clone repo
git clone git@github.com:zchen24/cisst_ros_integration.git 
# build 
cd ..
catkin_make
```
* download & compile dvrk\_research\_kit\_ros

```
# cd to catkin ws src dir
cd /PATH/TO/CATKIN_WS/src
# clone repo
git clone git@github.com:zchen24/dvrk_research_kit_ros.git 
# build 
cd ..
catkin_make
```

# How to run the code
* **simulation:** see dvrk_teleop/README.md details
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
  * MTM + PSM cat model & meshes
  * rviz config
  * lauch files
* dvrk_teleop:
  * dummy teleop component with a Qt GUI
* dvrk_robot: 
  * main file to fire robot, publish & subscribe ros topics 
* dvrk\_joint\_publisher:
  * puslish named joint states (position + velocity)
  
