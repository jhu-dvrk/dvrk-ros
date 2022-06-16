#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2015-02-22

# (C) Copyright 2015-2020 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_arm_test.py <arm-name>

import dvrk
import math
import sys
import select
import tty
import termios
import threading
import rospy
import numpy
import argparse

import psm_calibration_cv

import os.path
import xml.etree.ElementTree as ET

# for local_query_cp
import cisst_msgs.srv


# for keyboard capture
def is_there_a_key_press():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


class ArmCalibrationApplication:
    # configuration
    def configure(self, robot_name, config_file, expected_interval, timeout, convergence_threshold):
        self.expected_interval = expected_interval
        self.config_file = config_file
        # check that the config file is good
        if not os.path.exists(self.config_file):
            sys.exit("Config file \"{%s}\" not found".format(self.config_file))

        # XML parsing, find current offset
        self.tree = ET.parse(config_file)
        root = self.tree.getroot()

        # find Robot in config file and make sure name matches
        xpath_search_results = root.findall("./Robot")
        if len(xpath_search_results) == 1:
            xmlRobot = xpath_search_results[0]
        else:
            sys.exit("Can't find \"Robot\" in configuration file {:s}".format(self.config_file))

        if xmlRobot.get("Name") == robot_name:
            serial_number = xmlRobot.get("SN")
            print("Successfully found robot \"{:s}\", serial number {:s} in XML file".format(robot_name, serial_number))
            robotFound = True
        else:
            sys.exit("Found robot \"{:s}\" while looking for \"{:s}\", make sure you're using the correct configuration file!".format(xmlRobot.get("Name"), robot_name))

        # now find the offset for joint 2, we assume there's only one result
        xpath_search_results = root.findall("./Robot/Actuator[@ActuatorID='2']/AnalogIn/VoltsToPosSI")
        self.xmlPot = xpath_search_results[0]
        print("Potentiometer offset for joint 2 is currently: {:s}".format(self.xmlPot.get("Offset")))

        self.arm = dvrk.psm(arm_name = robot_name,
                            expected_interval = expected_interval)

        # Calibration parameters
        self.calibration_timeout = timeout
        self.calibration_convergence_threshold = convergence_threshold*1e-3 # mm to m

    def home(self):
        print('Enabling...')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print('Homing...')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')

        # get current joints just to set size
        print('Moving to zero position...')
        goal = numpy.copy(self.arm.setpoint_jp())
        goal.fill(0)
        self.arm.move_jp(goal).wait()
        self.arm.jaw.move_jp(numpy.array([0.0])).wait()
        # identify depth for tool j5 using forward kinematics
        local_query_cp = rospy.ServiceProxy(self.arm.namespace() + '/local/query_cp', cisst_msgs.srv.QueryForwardKinematics)
        request = cisst_msgs.srv.QueryForwardKinematicsRequest()
        request.jp.position = [0.0, 0.0, 0.0, 0.0]
        response = local_query_cp(request)
        self.q2 = response.cp.pose.position.z
        print("Depth required to position O5 on RCM point: {0:4.2f}mm".format(self.q2 * 1000.0))


    # get safe range of motion from user
    def find_range(self):
        print("Finding range of motion for joint {}".format(self.swing_joint))
        print("Move the arm manually (pressing the clutch) to find the maximum range of motion for the first joint (left to right motion).")
        print("    - press 'd' when you're done")
        print("    - press 'q' to abort")

        self.min = math.radians( 180.0)
        self.max = math.radians(-180.0)

        self.done = False

        # termios settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                if is_there_a_key_press():
                    char = sys.stdin.read(1)
                    if char == 'd':
                        self.done = True
                    elif char == 'q':
                        self.ok = False

                if not self.ok:
                    sys.exit("\n\nCalibration aborted by user")

                if self.done:
                    break

                # get measured joint values
                pose = self.arm.measured_jp()
                self.max = max(pose[self.swing_joint], self.max)
                self.min = min(pose[self.swing_joint], self.min)

                # display current range
                sys.stdout.write("\rRange: [{:02.2f}, {:02.2f}]".format(math.degrees(self.min), math.degrees(self.max)))
                sys.stdout.flush()

                # sleep
                rospy.sleep(self.expected_interval)

            # pass range of motion to vision tracking
            self.tracker.set_motion_range(self.max - self.min)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


    def move_to_start(self):
        goal_pose = numpy.copy(self.arm.setpoint_jp())
        goal_pose.fill(0)
        goal_pose[self.swing_joint] = self.min + 0.5 * (self.max - self.min)
        goal_pose[2] = self.q2 # to start close to expected RCM

        # rotate so vision target (joint 05) is facing user/camera
        if self.swing_joint == 0:
            goal_pose[3] = math.radians(90.0)
        else:
            goal_pose[3] = math.radians(0.0)

        # move to starting position
        self.arm.move_jp(goal_pose).wait()
        return goal_pose


    # get camera-relative target position at two arm poses to
    # establish camera orientation and scale (pixel to meters ratio)
    # exploratory_range is distance in meters to move arm
    def get_camera_jacobian(self, exploratory_range=0.016):
        print("\n\nMeasuring the orientation/scale of the camera")
        goal_pose = self.move_to_start()

        # move arm down from RCM by half of exploratory range
        goal_pose[2] = self.q2 + 0.5*exploratory_range
        self.arm.move_jp(goal_pose).wait()

        # get camera position of target (joint 05)
        print('Please click the target on the screen to aid target acquisition')
        ok, point_one = self.tracker.acquire_point()
        if not ok:
            print("Camera measurement failed")
            return False, None

        # slow down speed of generated trajectories - helps CV not to lose track of target
        self.arm.trajectory_j_set_ratio(0.03)

        # move arm up from RCM by half
        goal_pose[2] = self.q2 - 0.5*exploratory_range
        self.arm.move_jp(goal_pose).wait()

        # restore normal trajectory speed
        self.arm.trajectory_j_set_ratio(1.0)

        # get camera position of target again
        ok, point_two = self.tracker.acquire_point()
        if not ok:
            print("Camera measurement failed")
            return False, None

        # use difference in points with known physical difference and direction
        # to measure orientation/scale of camera
        point_difference = point_one - point_two
        scale = exploratory_range/numpy.linalg.norm(point_difference)
        orientation = point_difference/numpy.linalg.norm(point_difference)
        jacobian = scale*orientation

        print("Camera measurement completed successfully")
        return True, jacobian

    # called by vision tracking whenever a good estimate of the current RCM offset is obtained
    # return value indicates whether arm was moved along calibration axis
    def update_correction(self, rcm_offset, radius):
        alpha = 0.25
        self.residual_error = alpha*numpy.dot(rcm_offset, self.jacobian)

        # Move at most 5 mm (0.005 m) in direction of estimated RCM
        correction_delta = math.copysign(min(math.fabs(self.residual_error), 0.005), self.residual_error)
        print("Estimated remaining calibration error: {:0.2f} mm".format(1000*self.residual_error))

        # Limit total correction to 20 mm
        if abs(self.correction + correction_delta) > 0.020:
            print("Can't exceed 20 mm correction, please manually improve calibration first!")
            return

        self.correction += correction_delta

    def apply_correction(self, correction):
        elapsed_time = rospy.Time.now() - self.start_time
        self.tracker.stop_rcm_tracking()
        self.arm.move_jp(self.goal_pose).wait()
        self.goal_pose[2] = self.q2 + correction
        # slow down speed of generated trajectories - helps CV not to lose track of target
        self.arm.trajectory_j_set_ratio(0.01)
        self.arm.move_jp(self.goal_pose).wait()
        # restore normal trajectory speed
        self.arm.trajectory_j_set_ratio(1.0)
        self.tracker.clear_history()
        self.tracker.rcm_tracking(self.update_correction)

        # "hide" the time elapsed while updating the translation calibration
        # so that arm doesn't try to jump while resuming periodic movement
        self.start_time = rospy.Time.now() - elapsed_time

    # auto-calibration routine for third joint
    def calibrate_third_joint(self):
        print("\n\nBeginning auto-calibration...")

        self.arm.trajectory_j_set_ratio(0.02)
        self.goal_pose = self.move_to_start()
        self.arm.trajectory_j_set_ratio(1.0)

        print("During calibation, the target should be highlighted in magenta - if it becomes green or the highlight disappears,")
        print("then the target has been lost. If this occurs, please click on it to resume calibration.")
        print("Press q to stop calibration, for example if the camera is moved accidentally")

        self.correction = 0.0
        previous_correction = self.correction
        self.residual_error = self.calibration_convergence_threshold+1
        self.tracker.rcm_tracking(self.update_correction)
        self.start_time = rospy.Time.now()

        swing_range = 0.5*(self.max - self.min)
        swing_start = 0.5*(self.max + self.min)

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())

            # move back and forth while tracker measures calibration error
            while True:
                if is_there_a_key_press():
                    char = sys.stdin.read(1)
                    if char == 'q':
                        self.ok = False
                if not self.ok:
                    print("\n\nCalibration aborted by user")
                    return

                dt = rospy.Time.now() - self.start_time
                t = dt.to_sec() / 1.0
                self.goal_pose[self.swing_joint] = swing_start + swing_range*math.sin(t)

                correction = self.correction
                if correction != previous_correction:
                    self.apply_correction(correction)
                    previous_correction = correction
                else:
                    self.goal_pose[2] = self.q2 + correction
                    self.arm.servo_jp(self.goal_pose)

                if dt.to_sec() > self.calibration_timeout:
                    self.tracker.stop()
                    print('\n\nCalibration failed to converge within {} seconds'.format(self.calibration_timeout))
                    print('Try adding diffuse lighting, or increase timeout')
                    return

                if math.fabs(self.residual_error) < self.calibration_convergence_threshold:
                    self.tracker.stop()
                    print("\n\nCalibration successfully converged, with residual error of <{} mm".format(1000*self.calibration_convergence_threshold))
                    break

                rospy.sleep(self.expected_interval)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


        # return to start position
        self.move_to_start()

        # now save the new offset
        old_offset = float(self.xmlPot.get("Offset")) / 1000.0 # convert from XML (mm) to m
        new_offset = old_offset - self.correction # add in meters
        self.xmlPot.set("Offset", str(new_offset * 1000.0))    # convert from m to XML (mm)
        self.tree.write(self.config_file + "-new")
        print("Old offset: {:2.2f}mm\nNew offset: {:2.2f}mm\n".format(old_offset * 1000.0, new_offset * 1000.0))
        print("Results saved in {:s}-new.".format(self.config_file))
        print("Restart your dVRK application with the new file and make sure you re-bias the potentiometer offsets!")
        print("To be safe, power off and on the dVRK PSM controller.")
        print("To copy the new file over the existing one: cp {:s}-new {:s}".format(self.config_file, self.config_file))


    def _on_quit(self):
        self.ok = False
        self.tracker.stop()

    def _on_enter(self):
        self.done = True

    # application entry point
    def run(self, swing_joint):
        try:
            self.ok = True
            self.swing_joint = swing_joint

            # initialize vision tracking
            self.tracker = psm_calibration_cv.RCMTracker()
            self.ok = self.tracker.start(self._on_enter, self._on_quit)
            if not self.ok:
                return

            self.home()

            # find safe range of motion for arm
            self.find_range()

            # camera orientation/scale measurement
            self.ok, self.jacobian = self.get_camera_jacobian()
            if not self.ok:
                print("Aborting calibration")
                return

            self.calibrate_third_joint()

        finally:
            self.tracker.stop()

if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('dvrk_arm_test', anonymous=True)
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    parser.add_argument('-c', '--config', type=str, required=True,
                        help = 'arm IO config file, i.e. something like sawRobotIO1394-xwz-12345.xml')
    parser.add_argument('-s', '--swing-joint', type=int, required=False,
                        choices=[0, 1], default=0,
                        help = 'joint use for the swing motion around RCM')
    parser.add_argument('-t', '--timeout', type=int, required=False, default=180,
                        help = 'calibration timeout in seconds')
    parser.add_argument('--threshold', type=int, required=False, default=0.1,
                        help = 'calibration convergence threshold in mm')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    print ('\nThis program can be used to improve the potentiometer offset for the third joint '
           'of the PSM arm (translation stage).  The goal is increase the absolute accuracy of the PSM.\n'
           'The main idea is to position a known point on the tool (joint 05)  where the RCM should be.  '
           'If the calibration is correct, the point shouldn\'t move while the arm is rocking from left to right.  '
           'For this application we\'re going to use the axis of the first joint of the wrist, i.e. the first joint '
           'at the end of the tool shaft.  To perform this calibration you need to remove the canulla otherwise you won\'t see'
           ' the RCM point. This tool will use the camera to automatically track the point we want to be the RCM, and attempt '
           ' to calibrate the translation joint based.\n\n'
           'You must first home your PSM and make sure a tool is engaged.\n'
           'You should also have a camera point at the end of the tool, within about 3-5 inches if possible.\n'
           'For the camera to accurately track the target joint, it must be painted with bright pink nail polish.\n'
           'Once this is done, there are three steps:\n\n'
           ' -1- find a safe range of motion for the rocking movement\n'
           ' -3- detemine orientation/scale of camera relative to PSM\n'
           ' -2- monitor the application while auto-calibration is performed for safety.\n\n')

    application = ArmCalibrationApplication()
    application.configure(args.arm, args.config, args.interval, args.timeout, args.threshold)
    application.run(args.swing_joint)

