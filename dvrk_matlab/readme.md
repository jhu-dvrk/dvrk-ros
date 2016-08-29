Matlab interface for the dVRK ROS topics
========================================

Interface class used to subscribe and publish to the dVRK (da Vinci
Research Kit) ROS topics.  To use this class, one must first start the
cisst/SAW based C++ controller with the ROS interfaces.

See in `dvrk-ros` (on github: https://github.com/jhu-dvrk/dvrk-ros),
package `dvrk_robot`, application `dvrk_console_json`.  To compile and
install dvrk-ros, see
https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild

The file `arm.m` contains the matlab interface to the dVRK ROS topics.
It also handles data conversion from ROS messages (Poses to 4x4
homogeneous transformations) as well as timer based wait for blocking
commands (i.e. wait for state transitions and end of trajectories).

Warning
=======

Matlab multithreading support is not fantastic.  These wrappers use
callbacks assuming that at one point or another Matlab will dequeue
the ROS messages.  This is unfortunately not the case if you hog the
CPU using a loop and perform continuous computations.  So if you have
a loop, make sure you add plenty of sleep/pause so the ROS
callbacks get called.

Even worse, if you create your own ROS subscribers and use the
`receive` command, this messes up with the callback since it is a
blocking command.

We don't have a good fix for now, if you find some tricks or even
better a proper fix to improve the situation please let us know using
either the github issues or the dVRK google group.

Usage
=====

You will need Matlab 2015a or higher with the Robotic System Toolbox: http://www.mathworks.com/products/robotics/.  To test on Matlab on the same computer:
 * Start the application `dvrk_console_json` or any application using the standard ROS dVRK topics outside Matlab
 * In Matlab:
   * Start ROS using `rosinit`
   * Find the robot name using `rostopic list`.  You should see a list of namespaces under `/dvrk/`.  For example `/dvrk/PSM1/status` indicates that the arm `PSM1` is connected.
   * Create an interface for your arm using: `r = arm('PSM1);`
   * `r` will have the following properties (e.g. PSM has 7 joints):
     * `position_desired: [4x4 double]`
     * `position_joint_desired: [7x1 double]`
     * `effort_joint_desired: [7x1 double]` (output of PID)
     * `position_current: [4x4 double]`
     * `position_joint_current: [7x1 double]`
     * `velocity_joint_current: [7x1 double]`
     * `effort_joint_current: [7x1 double]` (based on motor current feedback)
   * To control the arm, one can do:
     * `r.home()`
     * `r.dmove_joint_one(-0.01, int8(3))` to move a single joint, be careful with radians and meters and joint index starts at 1 (not like C/C++)
     * `r.dmove_joint([-0.01, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0])` to move all joints (radians and meters)
     * `v = [0.0, 0.0, 0.01];`
       `r.dmove_translation(v)` to move in cartesian space, translation only.
     * Similar commands without the `dmove` prefix exist (start with `move`) and allow you to use absolute positions

Notes
=====

* There are also classes derived from `arm.m` for arm specific features, e.g. `mtm.m`, `psm.m`.
* If you're using two computers on the same subnet, make sure your resolv.conf or whatever file is used to resolve the full computer name is set so you can ping each computer using the short name (e.g. don't ping lcsr-dv-stereo.hwcampus.jhu.edu, ping lcsr-dv-stereo).  Otherwise, ROS might show you the topics but will still not be able to publish or subscribe.  On some systems you might even have to hard code the IP addresses of the remote host on both the "client" and "server" in the `hosts` file (google will tell you where to find the equivalent of `/etc/hosts` on different OSs).
* If you modify the `arm.m` file, send us a pull request or use the dVRK google group to let us know.  Your contributions are always welcome!
* The current code doesn't do a great job at cleaning up.  You might have to use `rosshutdown` and delete your robot instances manually.
* To create 4x4 matrices, take a look at the matlab commands `axang2tform`, `eul2tform`, ...
