dvrk_arm
========

# Install

See instructions from https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild

# How to run the code

To start a single arm, use the ROS scripts, e.g.
```bash
roslaunch dvrk_arm jhu-dv-ecm.launch
```

To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
```bash
rosrun dvrk_arm dvrk_arm_test.py
```
