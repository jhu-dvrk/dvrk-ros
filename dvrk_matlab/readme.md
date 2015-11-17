Matlab interface for the dVRK ROS topics
========================================

Interface class used to subscribe and publish to the dVRK (da Vinci Research Kit) ROS topics.  To use this class, one must first start the cisst/SAW based C++ controller with the ROS interfaces.

See in `dvrk-ros` (on github: https://github.com/jhu-dvrk/dvrk-ros), package `dvrk_robot`, application `dvrk_console_json`.  To compile and instal dvrk-ros, see https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild

The file robot.m contains the matlab interface to the dVRK ROS topics.  It also handles data conversion from ROS messages (Poses to 4x4 homogeneous transformations) as well as timer based wait for blocing commands (i.e. wait for state transitions and end of trajectories).

Usage
=====

You will need Matlab 2015a or higher with the Robotic System Toolbox: http://www.mathworks.com/products/robotics/.  To test on Matlab on the same computer:
 * Start the application `dvrk_console_json` or any application using the standard ROS dVRK topics outside Matlab
 * In Matlab:
   * Start ROS using `rosinit`
   * Find the robot name using `rostopic list`.  You should see a list of namespaces under `/dvrk/`.  For example `/dvrk/PSM1/status` indicates that the arm `PSM1` is connected.
   * Create an interface for your arm using: `r = robot('PSM1);`
   * `r` will have the following properties:
     * `position_cartesian_desired: [4x4 double]`
     * `position_joint_desired: [7x1 double]`
     * `position_cartesian_current: [4x4 double]`
     * `position_joint_current: [7x1 double]`
   * To control the arm, one can do:
     * `r.home()`
     * `r.delta_joint_move_single(int8(3), -0.01)` to move a single joint, be careful with radians and meters
     * `r.delta_joint_move([-0.01, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0])` to move all joints (radians and meters)
     * `v = [0.0, 0.0, 0.01];`
       `r.delta_cartesian_move_translation(v)` to move in cartesian space, translation only.
     * Similar commands without the `delta` prefix exist and allow you to use absolute positions

Notes
=====

* If you modify the `robot.m` file, send us a pull request or use the dVRK google group.  Your contributions are always welcome!
* The current code doesn't do a great job at cleaning up.  you might have to use `rosshutdown` and delete your robot instances manually.
* To create 4x4 matrices, take a look at the matlab commands `axang2tform`, `eul2tform`, ...
