Change log
==========

1.7.0 (2019-04-09)
==================

* API changes:
  * ROS namespaces are now relative, `dvrk_console_json` uses the namespace `dvrk/`, not `/dvrk/`.
* Deprecated features:
  * Old deprecated examples have been removed
* New features:
  * Support Python3
  * Log file (`cisstLog.txt`) is now timestamped
  * ROS topics to:
    * Set velocity/acceleration ratio for trajectory generation
    * Set and check active teleop PSM components
* Bug fixes:
  * Fixed issue with `coag` being hidden if used for operator present
  * Launch files use `xacro`, not the deprecated `xacro.py`

1.6.0 (2018-05-16)
==================

* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * dvrk_robot:
    * Add tf2 support
    * Multiple threads used, one for publishers, one for tf2 and one for subscribers.  This reduces latency on all subscribers
    * Publishes interval statistics for IO component as well as ROS bridges
    * SUJ joint state subscriber when in simulation mode
    * Added psm `set_effort_jaw` subscriber
  * Python:
    * Added PSM effort test program
    * Added cartesian impedance MTM test program
  * dVRK logo available in STL format for RViz and Gazebo
  * video.md: added some documentation and launch files for DeckLink frame grabbers
* Bug fixes:
  * Fixed missing latch on event publishers
  * `dvrk_console_json` should now quit on ctrl+c


1.5.0 (2017-11-07)
==================

* API changes:
  * dvrk_robot:
    * Update to match new arm state machine
    * Joint commands are now using joints for kinematics only.  On PSM, jaw is not the last joint anymore, it has its separate topics
    * PSM jaw and MTM gripper now use joint state to report position/velocity/effort
    * Better support for console/footpedals events
  * Python:
    * Added blocking/non blocking flag for move commands using trajectory generation
  * Matlab:
    * Added flag to send direct move commands (vs. trajectory goals)
    * Removed all callbacks since these tended to block the interpreter, now use getters to get latest value received.  Getters return timestamp as well.
    * Added ecm.m, console.m and teleop_psm.m
    * Code factorization for all conversion methods
  * Models/URDF:
    * PSM now subscribes to joint state for jaw to control last joint
    * Added crude models for 5mm tools
    * Fixed some xacro warnings
* Deprecated features:
  * None
* New features:
  * dvrk_robot:
    * Teleop now has an event to track if PSM is folowing master
    * SUJ publishes joint state
    * Publishes io interval statistics
    * Added topics for cartesian impedance controller + examples
    * Support for some topics when using generic MTMs (Falcon/ForceDimension)
  * Python:
    * Added test/example programs in dvrk_python/scripts
    * Added subscribers for jacobians, set effort joint
  * Matlab:
    * Added test/example programs in dvrk_matlab/test
    * Added subscribers for jacobians, set effort joint
* Bug fixes:
  * dvrk_calibrate_potentiometers now uses arm.py


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
