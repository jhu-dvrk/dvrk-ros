Change log
==========

1.4.0 (2016-08-31)
==================

* API changes:
  * Python:
    * API uses numpy arrays for joints and PyKDL for 3D commands, no more Python arrays/lists
    * Import as a package using `import dvrk`
    * Added base class arm.py with common features as well as mtm.py, ecm.py, psm.py for arm specific topics
    * Added wrappers console.py, suj.py and psm-teleop.py for corresponding ROS topics
  * Matlab:
    * API more closely matches Python interface
    * Use with CAUTION, it works fine in interactive mode but in a script, Matlab tends to never call the ROS spin function and the callbacks are failing.  See more comments in the dvrk_matlab readme.md
* Deprecated features:
  * Message types have changed to take adavantage of Stamped data type (mostly for timestamps).  See compatibility mode in new features.
  * dvrk_kinematics, dvrk_teleop and rqt_dvrk are now deprecated, still available in directory `deprecated`
* New features:
  * dvrk_robot:
    * Added many topics to match more closely the C++ available commands (see dVRK API wiki page)
    * Added compatibility mode to maintain old topic names and message types.  For example, `dvrk_console_json` has the option `--compatibility <value> : compatibility mode, e.g. "v1_3_0", "v1_4_0" (optional)`
* Bug fixes:
  * Better timestamps (cisst-ros new feature)

1.3.0 (2016-01-08)
==================

* API changes:
  * Python: now import as a package using `import dvrk_python.robot`
* Deprecated features:
  * None
* New features:
  * Potentiometer calibration script: `dvrk_robot/scripts/dvrk_calibrate_potentiometers.py`
  * dvrk_robot: added low level IO data collection for pots/joints/actuators (see potentiometer calibration wiki)
  * dvrk_robot: added set_wrench topics
  * dvrk_robot: console supports kinematic simulation with RViz (optional)
* Bug fixes:
  * None

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
