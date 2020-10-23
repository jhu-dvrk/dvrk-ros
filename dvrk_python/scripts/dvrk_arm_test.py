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
import time
import rospy
import numpy
import PyKDL
import argparse

# print with node id
def print_id(message):
    print('%s -> %s' % (rospy.get_caller_id(), message))

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name, expected_interval):
        print_id('configuring dvrk_arm_test for %s' % robot_name)
        self.expected_interval = expected_interval
        self.arm = dvrk.arm(arm_name = robot_name,
                            expected_interval = expected_interval)

    # homing example
    def home(self):
        print_id('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print_id('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print_id('move to starting position')
        goal = numpy.copy(self.arm.setpoint_jp())
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        goal.fill(0)
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2')
            or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            goal[2] = 0.12
        # move and wait
        print_id('moving to starting position')
        self.arm.wait_while_busy(self.arm.move_jp(goal))
        # try to move again to make sure waiting is working fine, i.e. not blocking
        print_id('testing move to current position')
        ts = self.arm.move_jp(goal)
        time.sleep(1.0) # add some artificial latency on this side
        self.arm.wait_while_busy(start_time = ts)
        print_id('home complete')

    # direct joint control example
    def servo_jp(self):
        print_id('starting servo_jp')
        # get current position
        initial_joint_position = numpy.copy(self.arm.setpoint_jp())
        print_id('testing direct joint position for 2 joints out of %i' % initial_joint_position.size)
        amplitude = math.radians(10.0) # +/- 10 degrees
        duration = 5  # seconds
        samples = duration / self.expected_interval
        # create a new goal starting with current position
        goal = numpy.copy(initial_joint_position)
        start = rospy.Time.now()
        for i in range(int(samples)):
            goal[0] = initial_joint_position[0] + amplitude *  math.sin(i * math.radians(360.0) / samples)
            goal[1] = initial_joint_position[1] + amplitude *  math.sin(i * math.radians(360.0) / samples)
            self.arm.servo_jp(goal)
            rospy.sleep(self.expected_interval)
        actual_duration = rospy.Time.now() - start
        print_id('servo_jp complete in %2.2f seconds (expected %2.2f)' % (actual_duration.to_sec(), duration))

    # goal joint control example
    def move_jp(self):
        print_id('starting move_jp')
        # get current position
        initial_joint_position = numpy.copy(self.arm.setpoint_jp())
        print_id('testing goal joint position for 2 joints out of %i' % initial_joint_position.size)
        amplitude = math.radians(10.0)
        # create a new goal starting with current position
        goal = numpy.copy(initial_joint_position)
        # first motion
        goal[0] = initial_joint_position[0] + amplitude
        goal[1] = initial_joint_position[1] - amplitude
        self.arm.wait_while_busy(self.arm.move_jp(goal))
        # second motion
        goal[0] = initial_joint_position[0] - amplitude
        goal[1] = initial_joint_position[1] + amplitude
        self.arm.wait_while_busy(self.arm.move_jp(goal))
        # back to initial position
        self.arm.wait_while_busy(self.arm.move_jp(initial_joint_position))
        print_id('move_jp complete')

    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        goal = numpy.copy(self.arm.setpoint_jp())
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2')
            or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            # set in position joint mode
            goal[0] = 0.0
            goal[1] = 0.0
            goal[2] = 0.12
            goal[3] = 0.0
            self.arm.wait_while_busy(self.arm.move_jp(goal))

    # direct cartesian control example
    def servo_cp(self):
        print_id('starting servo_cp')
        self.prepare_cartesian()

        # create a new goal starting with current position
        initial_cartesian_position = PyKDL.Frame()
        initial_cartesian_position.p = self.arm.setpoint_cp().p
        initial_cartesian_position.M = self.arm.setpoint_cp().M
        goal = PyKDL.Frame()
        goal.p = self.arm.setpoint_cp().p
        goal.M = self.arm.setpoint_cp().M
        # motion parameters
        amplitude = 0.05 # 5 cm
        duration = 5  # 5 seconds
        samples = duration / self.expected_interval
        start = rospy.Time.now()
        for i in range(int(samples)):
            goal.p[0] =  initial_cartesian_position.p[0] + amplitude *  math.sin(i * math.radians(360.0) / samples)
            goal.p[1] =  initial_cartesian_position.p[1] + amplitude *  math.sin(i * math.radians(360.0) / samples)
            self.arm.servo_cp(goal)
            # check error on kinematics, compare to desired on arm.
            # to test tracking error we would compare to
            # current_position
            setpoint_cp = self.arm.setpoint_cp()
            errorX = goal.p[0] - setpoint_cp.p[0]
            errorY = goal.p[1] - setpoint_cp.p[1]
            errorZ = goal.p[2] - setpoint_cp.p[2]
            error = math.sqrt(errorX * errorX + errorY * errorY + errorZ * errorZ)
            if error > 0.002: # 2 mm
                print_id('Inverse kinematic error in position [%i]: %s (might be due to latency)' % (i, error))
            rospy.sleep(self.expected_interval)
        actual_duration = rospy.Time.now() - start
        print_id('servo_cp complete in %2.2f seconds (expected %2.2f)' % (actual_duration.to_sec(), duration))

    # direct cartesian control example
    def move_cp(self):
        print_id('starting move_cp')
        self.prepare_cartesian()

        # create a new goal starting with current position
        initial_cartesian_position = PyKDL.Frame()
        initial_cartesian_position.p = self.arm.setpoint_cp().p
        initial_cartesian_position.M = self.arm.setpoint_cp().M
        goal = PyKDL.Frame()
        goal.p = self.arm.setpoint_cp().p
        goal.M = self.arm.setpoint_cp().M

        # motion parameters
        amplitude = 0.05 # 5 cm

        # first motion
        goal.p[0] =  initial_cartesian_position.p[0] - amplitude
        goal.p[1] =  initial_cartesian_position.p[1]
        self.arm.wait_while_busy(self.arm.move_cp(goal))
        # second motion
        goal.p[0] =  initial_cartesian_position.p[0] + amplitude
        goal.p[1] =  initial_cartesian_position.p[1]
        self.arm.wait_while_busy(self.arm.move_cp(goal))
        # back to initial position
        goal.p[0] =  initial_cartesian_position.p[0]
        goal.p[1] =  initial_cartesian_position.p[1]
        self.arm.wait_while_busy(self.arm.move_cp(goal))
        # first motion
        goal.p[0] =  initial_cartesian_position.p[0]
        goal.p[1] =  initial_cartesian_position.p[1] - amplitude
        self.arm.wait_while_busy(self.arm.move_cp(goal))
        # second motion
        goal.p[0] =  initial_cartesian_position.p[0]
        goal.p[1] =  initial_cartesian_position.p[1] + amplitude
        self.arm.wait_while_busy(self.arm.move_cp(goal))
        # back to initial position
        goal.p[0] =  initial_cartesian_position.p[0]
        goal.p[1] =  initial_cartesian_position.p[1]
        self.arm.wait_while_busy(self.arm.move_cp(goal))
        print_id('move_cp complete')

    # main method
    def run(self):
        self.home()
        self.servo_jp()
        self.move_jp()
        self.servo_cp()
        self.move_cp()

if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('dvrk_arm_test', anonymous=True)
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    application = example_application()
    application.configure(args.arm, args.interval)
    application.run()
