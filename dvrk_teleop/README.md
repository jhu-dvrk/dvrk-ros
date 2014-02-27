dvrk teleop 
====================
This package provides a teleop node and Qt GUI for simulation purpose. 

# Todo
* full system suppor ? not sure 

# How to run 
```sh
# roslaunch 
roslaunch dvrk_teleop test_teleop.launch
```

# List of Files 
* scripts: python test files, deprecate do NOT use
* include/src: 
  * mtsTeleop: class with teleop control
  * dvrkTeleopQWidget: GUI class
  * teleop.cpp: main file without GUI
  * teleopGUI.cpp: main file with GUI
