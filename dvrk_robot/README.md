dvrk robot 
==================
This package contains programs, which fires real robot, publishes and subscribes ROS topics 

# Depends 
* cisst 
* cisst\_ros\_integration 

# How to Run 
```sh
# create config folder
roscd dvrk_robot
mkdir config

# copy config fils (io/pid/kinematics)
cp /PATH/TO/sawIntuitiveResearchKit/share/*.* config 

# launch 
roslaunch dvrk_robot test_dvrk_robot.launch 

# wait ... wait ... wait ... 
# when programs all started 

# set robot state to HOME
rostopic pub -1 /dvrk_mtm/set_robot_state std_msgs/String Home

# set robot state to Gravity Compensation
rostopic pub -1 /dvrk_mtm/set_robot_state std_msgs/String Gravity
```

This program fires MTMR, uses rviz to visualize the robot.  
It demonstrates:
  * how to use mtsROSBridge 
  * how to puslish robot state to ROS topics
  * how to subscribe to ROS topics 
