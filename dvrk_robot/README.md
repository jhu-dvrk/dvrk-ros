dvrk robot 
==========

This package contains programs, which fires real robot, publishes and
subscribes ROS topics.  You can either run individual PIDs for any MTM
or PSM or run the whole system in TeleOp mode.

# Depends  
* ciss-ros 

IMPORTANT: You first need to make sure you have all your configuration
files ready, very likely in
~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share.  You don't
need to copy your configuration files back and forth anymore.

# How to Run

For the Qt based application without rviz:
  rosrun dvrk_robot dvrk_console_json -j <path_to_your_console_config.json>

We also provide a launch script for single arm using rviz (you need to provide your own console_<arm>.json file):
  roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 config:=/home/adeguet1/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-dVRK/console-PSM1.json

# Using the ROS topics

The best way to figure how to use the ROS topics is to look at the
files dvrk_python/src/robot.py and dvrk_matlab/robot.m.

**set robot state to HOME**  
rostopic pub -1 /dvrk_mtm/set_robot_state std_msgs/String Home

**set robot state to Gravity Compensation**  
rostopic pub -1 /dvrk_mtm/set_robot_state std_msgs/String Gravity

### Topic Names: 
https://github.com/jhu-dvrk/dvrk-ros/wiki/ROS-Topic-Interface

# Note
With this package, you no longer need the programs such as
* sawIntuitiveResearchKitQtPID -i ... -a ... -n ....
* sawIntuitiveResearchKitQtTeleOperationJSON -j two-arms.json


