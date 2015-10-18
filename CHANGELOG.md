Change log
==========

1.1.1 (2015-10-18)
==================

* Change log file created
* API changes:
  * Removed joint_publisher python code, now use topics directly from dVRK C++ stack (so much cleaner!)
  * Config file name, urdf and rviz, use dVRK C++ arm naming convention, i.e. PSM1, not psm_one, ...
  * Updated al urdf code to handle arm name as parameter (e.g. PSM1, PSM2)
* Deprecated features:
  * None
* New features:
  * dvrk_python/src/robot.py can be used a a simple Python API hiding all the ROS topics/data types 
  * dvrk_matlab/robot.m can be used a a simple Matlab API hiding all the ROS topics/data types 
* Bug fixes:
  * fixed urdf for PSM jaw angle
  * ...
