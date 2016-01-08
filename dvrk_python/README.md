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
from dvrk_python.robot import *
mtml = robot('MTML')
mtml.home()
mtml.get_current_joint_position()
...
```
