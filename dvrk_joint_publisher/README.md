dvrk_joint_publisher
==========
This package converts raw joint position and velocity values to JointStates with
name, position and velocity properly populated.

# File 
* scripts/mtm\_joint\_publisher.py
* scripts/psm\_joint\_publisher.py
  * credit Adnan Munawar
  
# Usage 

```sh
# NOTE robot_prefix should be consistant with 
# your robot urdf prefix defined in xacro file 
# For example 
#  dvrk_model/model/psm_one.urdf.xacro should use "one_"
rosrun dvrk_robot mtm_joint_publisher robot_prefix
```

See also dvrk\_robot/launch/test\_mtm\_ros.launch
