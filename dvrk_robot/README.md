dvrk robot 
==================
This package contains programs, which fires real robot, publishes and subscribes ROS topics.
You can either run individual PIDs for any MTM or PSM or run the whole system in TeleOp mode.

# Depends  
* ciss-ros 

IMPORTANT:
and the configuration files for MTMs/PSMs etc ... They would most probably be located in 
~/dev/cisst/source/saw/applications/sawIntuitiveResearchKit/share if you followed the custom
installation for CISST libraries from JHU. For your convinience, copy these files into a folder 
called /config in dvrk_robot directory (the directory this README file is located in). The launch files
in the /launch folder use this config folder, so don't forget to rename the configuration file
names in the launch files based on your DVRK configuration files that you just copied to the 
/config folder.

# How to Run 
roslaunch dvrk_robot test_dvrk_mtm.launch

roslaunch dvrk_robot test_dvrk_psm.launch


This will launch the Qt Application and for running the PSM/MTM and the Joint Publisher Slider GUI. You can 
use the sliders to move the joints, use caution to avoid collisions. You can then use rostopic list
to see all the topics that are available for getting/setting joint positions/torques/cartesians etc from 
command-line or from ROS nodes ..

to run the whole teleop program

rosrun dvrk_robot dvrk_full_ros -j {path to config folder}/two-arms.json


# create config folder
as explained above
* roscd dvrk_robot
* mkdir config

# copy config fils (io/pid/kinematics)
cp /PATH/TO/sawIntuitiveResearchKit/share/*.* config 


# set robot state to HOME
rostopic pub -1 /dvrk_mtm/set_robot_state std_msgs/String Home

# set robot state to Gravity Compensation
rostopic pub -1 /dvrk_mtm/set_robot_state std_msgs/String Gravity

# Topics for setting robot joint positions/end-effector cartesian pose
* **(sensor_msgs/JointState:positions)**
* /dvrk_psm/set_position_joint
* /dvrk_mtm/set_position_joint
* /dvrk_psm/set_position_joint
* /dvrk_mtm/set_position_joint

# Topics for getting robot joint positions/end-effector cartesian pose

* /dvrk_psm/joint_position_current
* /dvrk_mtm/joint_position_current
* /dvrk_psm/joint_cartesian_current
* /dvrk_mtm/joint_cartesian_current

# Topics for setting robot joint efforts
* **(sensor_msgs/JointState:efforts)**
* /dvrk_psm/set_joint_effort
* /dvrk_mtm/set_joint_effort

# Topics for getting robot joint efforts

* /dvrk_psm/joint_effort_current
* /dvrk_mtm/joint_effort_current

# Note
With this package, you no longer need the programs such as
* sawIntuitiveResearchKitQtPID -i ... -a ... -n ....
* sawIntuitiveResearchKitQtTeleOperationJSON -j two-arms.json

to run the arm pids or the entire teleop programs. The programs compiled with this 
package provide the same core functionality as the programs listed above and add
ros interface directly in the source code.


