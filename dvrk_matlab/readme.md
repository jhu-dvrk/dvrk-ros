Matlab interface for the dVRK ROS topics
========================================

Interface class used to subscribe and publish to the dVRK (da Vinci Research Kit) ROS topics.  To use this class, one must first start the cisst/SAW based C++ controller with the ROS interfaces.

See in `dvrk-ros` (on github: https://github.com/jhu-dvrk/dvrk-ros), package `dvrk_robot`, application `dvrk_console_json`.  To compile and
install dvrk-ros, see https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild.

To configure your Matlab path and make sure the CRTK custom messages support is all set, please read [CRTK Matlab client README](https://github.com/collaborative-robotics/crtk_matlab_client).

The class `dvrk.arm` contains the matlab interface to the dVRK ROS topics.  It also handles data conversion from ROS messages (Poses to 4x4 homogeneous transformations) as well as timer based wait for blocking commands (i.e. wait for state transitions and end of trajectories).

Warning
=======

The default classes in the dVRK Matlab package subscribe to all the dVRK topics we can subscribe to.  This is convenient to test your code or use in an interactive shell.  For a user application, it is recommended to create your own `arm` class and only add the features you need.  This will reduce the amount of network messages and computing load on both sides (dVRK and Matlab).  You should likely copy the file `+dvrk/arm.m` and remove the features you don't need.

Usage
=====

You will need Matlab 2015a or higher with the Robotic System Toolbox: http://www.mathworks.com/products/robotics/.  To test on Matlab on the same computer:
 * Start the application `dvrk_console_json` or any application using the standard ROS dVRK topics outside Matlab
 * In Matlab:
   * See examples in `examples` directory!
   * Start ROS using `rosinit`
   * Find the robot name using `rostopic list`.  You should see a list of namespaces for each arm.  For example `/MTML/measured_js` indicates that the arm `MTML` is available.
   * Create an interface for your arm using: `r = arm('MTML);`
   * All units are SI (radians and meters)
   * `r` will have the following methods (e.g. MTM has 7 joints):
     * `r.setpoint_cp()` [4x4 floats]
     * `[position, velocity, effort] = r.setpoint_js()` [n floats, empty, n floats] (output of PID)
     * `r.measured_cp()` [4x4 floats]
     * `[position, velocity, effort] = r.setpoint_js()` [n floats, n floats, n floats] (based on encoder and motor current feedback)
   * To control the arm, one can do:
     * `r.enable(60)` (with timeout)
     * `r.home(30)`
     * `r.servo_jp()` (takes n joint positions sets the setpoint)
     * `r.move_jp()` to move all joints
     * `r.wait_while_busy(r.move())` move and wait until move is complete
     * `r.servo_cp()` and `r.move_cp()` take 4x4 frame

Notes
=====

* There are also classes derived from `arm.m` for arm specific features, e.g. `mtm.m`, `psm.m`.
* If you're using two computers on the same subnet, make sure your resolv.conf or whatever file is used to resolve the full computer name is set so you can ping each computer using the short name (e.g. don't ping lcsr-dv-stereo.hwcampus.jhu.edu, ping lcsr-dv-stereo).  Otherwise, ROS might show you the topics but will still not be able to publish or subscribe.  On some systems you might even have to hard code the IP addresses of the remote host on both the "client" and "server" in the `hosts` file (google will tell you where to find the equivalent of `/etc/hosts` on different OSs).
* If you modify the `arm.m` file, send us a pull request or use the dVRK google group to let us know.  Your contributions are always welcome!
* The current code doesn't do a great job at cleaning up.  You might have to use `rosshutdown` and delete your robot instances manually.
* To create 4x4 matrices, take a look at the matlab commands `axang2tform`, `eul2tform`, ...
