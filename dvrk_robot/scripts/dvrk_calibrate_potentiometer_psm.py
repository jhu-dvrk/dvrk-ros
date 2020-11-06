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
import rospy
import numpy
import argparse

import os.path
import xml.etree.ElementTree as ET

# for local_query_cp
import cisst_msgs.srv

# for keyboard capture
def is_there_a_key_press():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name, config_file, expected_interval):
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
        self.arm.wait_while_busy(self.arm.move_jp(goal))
        self.arm.wait_while_busy(self.arm.jaw.move_jp(numpy.array([0.0])))
        # identify depth for tool j5 using forward kinematics
        local_query_cp = rospy.ServiceProxy(self.arm.namespace() + '/local/query_cp', cisst_msgs.srv.QueryForwardKinematics)
        request = cisst_msgs.srv.QueryForwardKinematicsRequest()
        request.jp.position = [0.0, 0.0, 0.0, 0.0]
        response = local_query_cp(request)
        self.q2 = response.cp.pose.position.z
        print("Depth required to position O5 on RCM point: {0:4.2f}mm".format(self.q2 * 1000.0))

    # find range
    def find_range(self):
        print('Finding range of motion for joint 0\n - move the arm manually (pressing the clutch)\n - press "d" when you\'re done\n - press "q" to abort')
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
                if p[0] > self.max:
                    self.max = p[0]
                elif p[0] < self.min:
                    self.min = p[0]
                # display current range
                sys.stdout.write('\rRange[%02.2f, %02.2f]' % (math.degrees(self.min), math.degrees(self.max)))
                sys.stdout.flush()
                # sleep
                rospy.sleep(self.expected_interval)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print('')

    # direct joint control example
    def calibrate_third_joint(self):
        # move to max position as starting point
        initial_joint_position = numpy.copy(self.arm.setpoint_jp())
        goal = numpy.copy(self.arm.setpoint_jp())
        goal.fill(0)
        goal[0] = self.max
        goal[2] = self.q2 # to start close to expected RCM
        goal[3] = math.radians(90.0) # so axis is facing user

        self.arm.wait_while_busy(self.arm.move_jp(goal))

        # parameters to move back and forth
        cos_ratio = (self.max - self.min) / 2.0

        # termios settings
        old_settings = termios.tcgetattr(sys.stdin)
        correction = 0.0
        try:
            tty.setcbreak(sys.stdin.fileno())
            start = rospy.Time.now()
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
                dt = rospy.Time.now() - start
                t = dt.to_sec() / 2.0
                goal[0] = self.max + cos_ratio * (math.cos(t) - 1.0)
                goal[2] = self.q2 + correction
                self.arm.servo_jp(goal)
                # display current offset
                sys.stdout.write('\rOffest = %02.2f mm' % (goal[2] * 1000.0))
                sys.stdout.flush()
                # sleep
                rospy.sleep(self.expected_interval)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print('')

        # now save the new offset
        oldOffset = float(self.xmlPot.get("Offset"))
        self.xmlPot.set("Offset", str(oldOffset + correction))
        self.tree.write(self.config_file + "-new")
        print('Results saved in {:s}-new. Restart your dVRK application with the new file!'.format(self.config_file))
        print('To copy the new file over the existing one: cp {:s}-new {:s}'.format(self.config_file, self.config_file))


    # main method
    def run(self):
        self.home()
        self.find_range()
        self.calibrate_third_joint()

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
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    application = example_application()
    application.configure(args.arm, args.config, args.interval)
    application.run()
