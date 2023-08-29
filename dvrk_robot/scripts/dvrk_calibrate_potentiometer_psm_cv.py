#!/usr/bin/env python

# Authors: Anton Deguet, Brendan Burkhart
# Date: 2015-02-22

# (C) Copyright 2015-2023 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_arm_test.py <arm-name>

import crtk
import dvrk

import math
import sys
import time
import select
import tty
import termios
import threading
import numpy
import argparse

import psm_calibration_cv

import os.path
import xml.etree.ElementTree as ET


# for keyboard capture
def is_there_a_key_press():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


class calibration_psm_cv:
    
    # configuration
    def __init__(self, ral, arm_name, config_file, expected_interval, timeout, convergence_threshold):
        self.expected_interval = expected_interval
        self.config_file = config_file
        # check that the config file is good
        if not os.path.exists(self.config_file):
            sys.exit('Config file "{:s}" not found'.format(self.config_file))

        # XML parsing, find current offset
        self.tree = ET.parse(config_file)
        root = self.tree.getroot()

        # find Robot in config file and make sure name matches
        xpath_search_results = root.findall('./Robot')
        if len(xpath_search_results) == 1:
            xmlRobot = xpath_search_results[0]
        else:
            sys.exit('Can\'t find "Robot" in configuration file {:s}'.format(self.config_file))

        if xmlRobot.get('Name') == arm_name:
            serial_number = xmlRobot.get('SN')
            print('Successfully found robot "{:s}", serial number {:s} in XML file'.format(arm_name, serial_number))
            robotFound = True
        else:
            sys.exit('Found robot "{:s}" while looking for "{:s}", make sure you\'re using the correct configuration file!'.format(xmlRobot.get('Name'), arm_name))

        # now find the offset for joint 2, we assume there's only one result
        xpath_search_results = root.findall("./Robot/Actuator[@ActuatorID='2']/AnalogIn/VoltsToPosSI")
        self.xmlPot = xpath_search_results[0]
        print('Potentiometer offset for joint 2 is currently: {:s}'.format(self.xmlPot.get('Offset')))

        self.arm = dvrk.psm(ral = ral,
                            arm_name = arm_name,
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
        cp = self.arm.forward_kinematics(numpy.array([0.0, 0.0, 0.0, 0.0]))
        self.q2 = cp.p.z()
        print('Depth required to position O5 on RCM point: {0:4.2f}mm'.format(self.q2 * 1000.0))


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
                time.sleep(self.expected_interval)

            # restrict range to center it at 0
            angle = min(math.fabs(self.min), math.fabs(self.max))
            self.min = -max(angle, self.min)
            self.max = min(angle, self.max)
            print("\nRange to be used: [{:02.2f}, {:02.2f}]".format(math.degrees(self.min), math.degrees(self.max)))
            # pass range of motion to vision tracking
            self.tracker.set_motion_range(self.max - self.min)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print('')


    # Move arm to middle of motion range and position joint 05 near RCM
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
        self.arm.trajectory_j_set_ratio(0.05)

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
    def update_correction(self, rcm_offset):
        self.residual_error = numpy.dot(rcm_offset, self.jacobian)
        # slow convergence at 0.25 mm
        convergence_rate = 0.325 if math.fabs(self.residual_error) < 0.00025 else 1.0

        # Move at most 5 mm (0.005 m) in direction of estimated RCM
        correction_delta = math.copysign(min(math.fabs(convergence_rate*self.residual_error), 0.005), self.residual_error)
        print("Estimated remaining calibration error: {:0.2f} mm".format(1000*self.residual_error))

        # Limit total correction to 20 mm
        if abs(self.correction + correction_delta) > 0.020:
            print("Can't exceed 20 mm correction, please manually improve calibration first!")
            return

        self.correction += correction_delta

    # Adjust translation stage to apply new calibration correction
    def apply_correction(self, correction):
        # stop tracking rcm, apply correction, re-start rcm tracking
        self.tracker.stop_rcm_tracking()
        self.goal_pose[2] = self.q2 + correction
        self.arm.move_jp(self.goal_pose).wait()
        self.tracker.clear_history()
        self.tracker.rcm_tracking(self.update_correction)

    # auto-calibration routine for third joint
    def calibrate_third_joint(self):
        print("\n\nBeginning auto-calibration...")

        # slow down arm, move to start position, restore normal speed
        self.arm.trajectory_j_set_ratio(0.05)
        self.goal_pose = self.move_to_start()
        self.arm.trajectory_j_set_ratio(1.0)

        print("During calibation, the target should be highlighted in magenta - if it becomes green or the highlight disappears,")
        print("then the target has been lost. If this occurs, please click on it to resume calibration.")
        print("Press q to stop calibration, for example if the camera is moved accidentally")

        self.correction = 0.0
        previous_correction = self.correction
        self.residual_error = self.calibration_convergence_threshold+1
        self.tracker.clear_history()
        self.tracker.rcm_tracking(self.update_correction)

        move_command_handle = None
        self.start_time = time.time()

        # Swing arm back and forth until timeout, convergence, or error/user abort
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            self.arm.trajectory_j_set_ratio(0.15)

            # move back and forth while tracker measures calibration error
            while True:
                if is_there_a_key_press():
                    char = sys.stdin.read(1)
                    if char == 'q':
                        self.ok = False
                if not self.ok:
                    print("\n\nCalibration aborted by user")
                    pose = self.arm.measured_jp()
                    self.arm.move_jp(pose).wait()
                    return

                # thread-safe check for change in calibration correction
                correction = self.correction
                if correction != previous_correction:
                    self.apply_correction(correction)
                    previous_correction = correction

                # Once arm has reached end of swing, set trajectory to swing back other way
                if move_command_handle is None or not move_command_handle.is_busy():
                    self.goal_pose[2] = self.q2 + correction
                    self.goal_pose[self.swing_joint] = self.max if self.goal_pose[self.swing_joint] == self.min else self.min
                    move_command_handle = self.arm.move_jp(self.goal_pose)

                time_elapsed = time.time() - self.start_time
                if time_elapsed.to_sec() > self.calibration_timeout:
                    self.tracker.stop()
                    print('\n\nCalibration failed to converge within {} seconds'.format(self.calibration_timeout))
                    print('Try adding diffuse lighting, increasing the range of motion, or increase the timeout')
                    return

                if math.fabs(self.residual_error) < self.calibration_convergence_threshold:
                    self.tracker.stop()
                    print("\n\nCalibration successfully converged, with residual error of <{} mm".format(1000*self.calibration_convergence_threshold))
                    break

                time.sleep(self.expected_interval)

        # Restore normal terminal behavior and normal arm speed
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self.arm.trajectory_j_set_ratio(1.0)

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

    # Exit key (q/ESCAPE) handler for GUI
    def _on_quit(self):
        self.ok = False
        self.tracker.stop()

    # Enter (or 'd') handler for GUI
    def _on_enter(self):
        self.done = True

    # application entry point
    def run(self, swing_joint, rom):
        try:
            self.ok = True
            self.swing_joint = swing_joint

            # initialize vision tracking
            self.tracker = psm_calibration_cv.RCMTracker()
            self.ok = self.tracker.start(self._on_enter, self._on_quit)
            if not self.ok:
                return

            self.home()

            if rom == 0:
                # find safe range of motion for arm
                self.find_range()
            else:
                self.max = math.radians(rom)
                self.min = math.radians(-rom)
                self.tracker.set_motion_range(self.max - self.min)

            # camera orientation/scale measurement
            self.ok, self.jacobian = self.get_camera_jacobian()
            if not self.ok:
                print("Aborting calibration")
                return

            # actual calibration
            self.calibrate_third_joint()

        finally:
            self.tracker.stop()

if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    parser.add_argument('-r', '--range', type=float, default=0.0,
                        help = 'range of motion, degrees')
    parser.add_argument('-c', '--config', type=str, required=True,
                        help = 'arm IO config file, i.e. something like sawRobotIO1394-xwz-12345.xml')
    parser.add_argument('-s', '--swing-joint', type=int, required=False,
                        choices=[0, 1], default=0,
                        help = 'joint use for the swing motion around RCM')
    parser.add_argument('-t', '--timeout', type=int, required=False, default=180,
                        help = 'calibration timeout in seconds')
    parser.add_argument('--threshold', type=int, required=False, default=0.1,
                        help = 'calibration convergence threshold in mm')
    args = parser.parse_args(argv)

    print ('\nThis program can be used to improve the potentiometer offset for the third joint '
           'of the PSM arm (translation stage).  The goal is increase the absolute accuracy of the PSM.\n'
           'The main idea is to position a known point on the tool (joint 05)  where the RCM should be.  '
           'If the calibration is correct, the point shouldn\'t move while the arm is rocking from left to right.  '
           'For this application we\'re going to use the axis of the first joint of the wrist, i.e. the first joint '
           'at the end of the tool shaft.  To perform this calibration you need to remove the canulla otherwise you won\'t see'
           ' the RCM point. This tool will use the camera to automatically track the point we want to be the RCM, and attempt '
           ' to calibrate the translation joint based.\n\n'
           'You must first home your PSM and make sure a tool is engaged.\n'
           'You should also have a camera point at the end of the tool, within about 2-4 inches if possible.\n'
           'For the camera to accurately track the target joint, it must be painted with bright pink nail polish.\n'
           'Once this is done, there are three steps:\n\n'
           ' -1- find a safe range of motion for the rocking movement\n'
           ' -2- detemine orientation/scale of camera relative to PSM\n'
           ' -3- monitor the application while auto-calibration is performed for safety.\n\n')

    ral = crtk.ral('dvrk_calibrate_potentiometer_psm_cv')
    application = calibration_psm_cv(ral, args.arm, args.config, args.interval, args.timeout, args.threshold)
    ral.spin_and_execute(application.run, args.swing_joint, args.range)
