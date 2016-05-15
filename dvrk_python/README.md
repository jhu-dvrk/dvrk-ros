## Introduction

This directory contains:
* dvrk_python module.  This modules defines the class `robot` which uses the dVRK ROS topics to communicate with the `dvrk_console_json` application included in the `dvrk_robot` ROS package.
* tutorial examples.  Python scripts using the `dvrk_python` module.

## Requirements

You will need to build the dVRK software stack using the ROS catkin build tools, see https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild

The `dvrk_python` package is included in the `dvrk_ros` git repository.  Please checkout the whole `dvrk_ros` repository and build it using the catkin build tools (don't use `catkin_make`).

You will also need some extra Python packages:
```sh
   sudo apt-get install ros-hydro-python-orocos-kdl ros-hydro-orocos-kinematics-dynamics ros-hydro-tf
```

## Runtime

You first need to start the dVRK console application:
```sh
  rosrun dvrk_robot dvrk_console_json -j your_console_config_file.json
```

Once the console is running, you can check which arms are available using the `rostopic` command:
```sh
  rostopic list
```

You should see one namespace per arm under `/dvrk`, e.g. `/dvrk/PSM1`, `/dvrk/MTML` and/or `/dvrk/ECM` based on your console configuration.

Then in Python:
```python
from dvrk.psm import *
# Create a Python proxy for PSM1, name must match ros namespace
p = arm('PSM1')

# You can home from Python
p.home()

# retrieve current info (numpy.array)
p.get_current_joint_position()
p.get_current_joint_velocity()
p.get_current_joint_effort()

# retrieve PID desired position and effort computed
p.get_desired_joint_position()
p.get_desired_joint_effort()

# retrieve cartesian current and desired positions
# PyKDL.Frame
p.get_desired_position()
p.get_current_position()

# move in joint space
# move is absolute (SI units)
# dmove is relative

# move a single joint, index starts at 0
p.dmove_joint_one(-0.05, 2) # move 3rd joint
p.move_joint_one(0.2, 0) # first joint

# move multiple joints
p.dmove_joint_some(numpy.array([-0.1, -0.1]), numpy.array([0, 1]))
p.move_joint_some(numpy.array([0.0, 0.0]), numpy.array([0, 1]))

# move all joints
p.dmove_joint(numpy.array([0.0, 0.0, -0.05, 0.0, 0.0, 0.0, 0.0]))
p.move_joint(numpy.array([0.0, 0.0, 0.10, 0.0, 0.0, 0.0, 0.0]))

# move in cartesian space
# there are only 2 methods available, dmove and move
# both accept PyKDL Frame, Vector or Rotation
p.dmove(PyKDL.Vector(0.0, 0.05, 0.0)) # 5 cm in Y direction 
p.move(PyKDL.Vector(0.0, 0.0, -0.05))

# save current orientation
old_orientation = p.get_desired_position().R

import math
r = PyKDL.Rotation()
r.DoRotX(math.pi * 0.25)
p.dmove(r)

p.move(old_orientation)
```
