## Introduction

This directory contains:
* dvrk_python module.  This modules defines the class `robot` which uses the dVRK ROS topics to communicate with the `dvrk_console_json` application included in the `dvrk_robot` ROS package.
* tutorial examples.  Python scripts using the `dvrk_python` module.

## Requirements

You will need to build the dVRK software stack using the ROS catkin build tools, see https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild

The `dvrk_python` package is included in the `dvrk_ros` git repository.  Please checkout the whole `dvrk_ros` repository and build it using the catkin build tools (don't use `catkin_make`).

The dVRK Python ROS client now relies on CRTK so you will also need the following packages:
* https://github.com/collaborative-robotics/crtk_msgs
* https://github.com/collaborative-robotics/crtk_python_client
If you followed the dVRK build instructions above, these packages are likely already installed in your workspace

## Runtime

You first need to start the dVRK console application:
```sh
  rosrun dvrk_robot dvrk_console_json -j your_console_config_file.json
```

Once the console is running, you can check which arms are available using the `rostopic` command:
```sh
  rostopic list
```

You should see one namespace per arm, e.g. `/PSM1`, `/MTML` and/or `/ECM` based on your console configuration.

Then in Python:
```python
import dvrk
# Create a Python proxy for PSM1, name must match ros namespace
p = dvrk.psm('PSM1')

# You can home from Python
p.home()

# retrieve current info (numpy.array)
p.measured_jp()
p.measured_jv()
p.measured_jf()

# retrieve PID desired position and effort computed
p.setpoint_jp()
p.setpoint_jf()

# retrieve cartesian current and desired positions
# PyKDL.Frame
p.measured_cp()
p.setpoint_cp()

# move in joint space
# move is absolute (SI units)

# move multiple joints
import numpy
p.move_jp(numpy.array([0.0, 0.0, 0.10, 0.0, 0.0, 0.0]))

# move in cartesian space
import PyKDL
p.move_cp(PyKDL.Vector(0.0, 0.0, -0.05))

# save current orientation
old_orientation = p.setpoint_cp().M

import math
r = PyKDL.Rotation()
r.DoRotX(math.pi * 0.25)
p.dmove(r)

p.move(old_orientation)
```

To apply wrenches on MTMs, start ipython and type the following commands while holding the MTM (otherwise the arm will start moving and might bang itself against the console and get damaged).

```python
# load and define the MTM
from dvrk import mtm
m = mtm('MTML')

# When True, force direction is constant.  Otherwise force direction defined in gripper coordinate system
m.set_wrench_body_orientation_absolute(True)

# about 2N force in y direction
m.body.servo_cf(numpy.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0]))

# lock the MTM wrist orientation
m.lock_orientation_as_is()

# turn gravity compensation on/off
m.set_gravity_compensation(True)

# turn off forces
self.arm.body.servo_cf(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
```
To access arm specific features (e.g. PSM, MTM, ...), you can use the derived classes `psm` or `mtm`.   For example `from dvrk.psm import *`.
