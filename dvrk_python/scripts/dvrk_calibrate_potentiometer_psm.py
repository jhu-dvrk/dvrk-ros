#!/usr/bin/env python

# Author: Anton Deguet
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
import numpy
import argparse

import os.path
import xml.etree.ElementTree as ET

# for keyboard capture
def is_there_a_key_press():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

# example of application using arm.py
class calibration_psm:

    # configuration
    def __init__(self, ral, arm_name, config_file, expected_interval = 0.01):
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

    # homing example
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

    # find range
    def find_range(self, swing_joint):
        if swing_joint == 0:
            print('Finding range of motion for joint 0\nMove the arm manually (pressing the clutch) to find the maximum range of motion for the first joint (left to right motion).\n - press "d" when you\'re done\n - press "q" to abort\n')
        else:
            print('Finding range of motion for joint 1\nMove the arm manually (pressing the clutch) to find the maximum range of motion for the second joint (back to front motion).\n - press "d" when you\'re done\n - press "q" to abort\n')

        self.min = math.radians( 180.0)
        self.max = math.radians(-180.0)
        done = False
        increment = numpy.copy(self.arm.setpoint_jp())
        increment.fill(0)

        # termios settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while not done:
                # process key
                if is_there_a_key_press():
                    c = sys.stdin.read(1)
                    if c == 'd':
                        done = True
                    elif c == 'q':
                        sys.exit('... calibration aborted by user')
                # get measured joint values
                p = self.arm.measured_jp()
                if p[swing_joint] > self.max:
                    self.max = p[swing_joint]
                elif p[swing_joint] < self.min:
                    self.min = p[swing_joint]
                # display current range
                sys.stdout.write('\rRange[%02.2f, %02.2f]' % (math.degrees(self.min), math.degrees(self.max)))
                sys.stdout.flush()
                # sleep
                time.sleep(self.expected_interval)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print('')

    # direct joint control example
    def calibrate_third_joint(self, swing_joint):
        print('\nAdjusting translation offset\nPress the keys "+" (or "=") and "-" or ("_") to adjust the depth until the axis 5 is mostly immobile (one can use a camera to look at the point)\n - press "d" when you\'re done\n - press "q" to abort\n')
        # move to max position as starting point
        initial_joint_position = numpy.copy(self.arm.setpoint_jp())
        goal = numpy.copy(self.arm.setpoint_jp())
        goal.fill(0)
        goal[swing_joint] = self.max
        goal[2] = self.q2 # to start close to expected RCM
        if swing_joint == 0:
            goal[3] = math.radians(90.0) # so axis is facing user
        else:
            goal[3] = math.radians(0.0)

        self.arm.move_jp(goal).wait()

        # parameters to move back and forth
        cos_ratio = (self.max - self.min) / 2.0

        # termios settings
        old_settings = termios.tcgetattr(sys.stdin)
        correction = 0.0
        try:
            tty.setcbreak(sys.stdin.fileno())
            start = time.time()
            done = False
            while not done:
                # process key
                if is_there_a_key_press():
                    c = sys.stdin.read(1)
                    if c == 'd':
                        done = True
                    elif c == 'q':
                        sys.exit('... calibration aborted by user')
                    elif c == '-' or c == '_':
                        correction = correction - 0.0001
                    elif c == '+' or c == '=':
                        correction = correction + 0.0001
                # move back and forth
                dt = time.time() - start
                t = dt / 2.0
                goal[swing_joint] = self.max + cos_ratio * (math.cos(t) - 1.0)
                goal[2] = self.q2 + correction
                self.arm.servo_jp(goal)
                # display current offset
                sys.stdout.write('\rCorrection = %02.2f mm' % (correction * 1000.0))
                sys.stdout.flush()
                # sleep
                time.sleep(self.expected_interval)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print('')

        # now save the new offset
        oldOffset = float(self.xmlPot.get('Offset')) / 1000.0 # convert from XML (mm) to m
        newOffset = oldOffset - correction                    # add in meters
        self.xmlPot.set('Offset', str(newOffset * 1000.0))    # convert from m to XML (mm)
        os.rename(self.config_file, self.config_file + '-backup')
        self.tree.write(self.config_file)
        print('Old offset: {:2.2f}mm\nNew offset: {:2.2f}mm\n'.format(oldOffset * 1000.0, newOffset * 1000.0))
        print('Results saved in {}. Restart your dVRK application to use the new file!'.format(self.config_file))
        print('Old file saved as {}-backup.'.format(self.config_file))

    # main method
    def run(self, swing_joint):
        self.home()
        self.find_range(swing_joint)
        self.calibrate_third_joint(swing_joint)

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
    parser.add_argument('-c', '--config', type=str, required=True,
                        help = 'arm IO config file, i.e. something like sawRobotIO1394-xwz-12345.xml')
    parser.add_argument('-s', '--swing-joint', type=int, required=False,
                        choices=[0, 1], default=0,
                        help = 'joint use for the swing motion around RCM')
    args = parser.parse_args(argv)

    print ('\nThis program can be used to improve the potentiometer offset for the third joint '
           'of the PSM arm (translation stage).  The goal is increase the absolute accuracy of the PSM.\n'
           'The main idea is to position a known point on the tool where the RCM should be.  '
           'If the calibration is correct, the point shouldn\'t move while the arm is rocking from left to right.  '
           'For this application we\'re going to use the axis of the first joint of the wrist, i.e. the first joint '
           'at the end of the tool shaft.  To perform this calibration you need to remove the canulla otherwise you won\'t see'
           ' the RCM point.   One simple way to track the motion is to use a camera and place the cursor where the axis is.\n\n'
           'You must first home your PSM and make sure a tool is engaged.  '
           'Once this is done, there are two steps:\n'
           ' -1- find a safe range of motion for the rocking movement\n'
           ' -2- adjust the depth so that the first hinge on the tool wrist is as close as possible to the RCM.\n\n')

    ral = crtk.ral('dvrk_calibrate_potentiometer_psm')
    application = calibration_psm(ral, args.arm, args.config, args.interval)
    ral.spin_and_execute(application.run, args.swing_joint)
