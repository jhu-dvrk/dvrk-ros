dvrk robot 
==========

This package contains programs, which fires real robot, publishes and
subscribes ROS topics.

# Depends  
* cisst-ros 

IMPORTANT: You first need to make sure you have all your configuration
files ready, very likely in
~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share.  You don't
need to copy your configuration files back and forth anymore.

# How to Run

For the Qt based application without rviz:
```sh
  rosrun dvrk_robot dvrk_console_json -j <path_to_your_console_config.json>
```

We also provide a launch script for single arm using RViz (you need to provide your own console_<arm>.json file):
```sh
  roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 config:=/home/<user_name>/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-dVRK/console-PSM1.json
```

One can also simulate one or more arms using the field `simulation` in your console-xyz.json.  See examples in the `sawIntuitiveResearchKit/share` directory:
```sh
  roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=ECM config:=/home/<user_name>/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/console-ECM_KIN_SIMULATED.json
```

# Using the ROS topics

The best way to figure how to use the ROS topics is to look at the
files dvrk_python/src/robot.py and dvrk_matlab/robot.m.
