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
# Run test script:
# > rosrun dvrk_python dvrk_psm_effort_test.py -a <arm-name>

import argparse
import crtk
import dvrk
import math
import numpy
import sys

if sys.version_info.major < 3:
    input = raw_input

# example of application using arm.py
class example_application:
    def __init__(self, ral, arm_name, expected_interval):
        print('configuring dvrk_psm_effort_test for {}'.format(arm_name))
        self.ral = ral
        self.expected_interval = expected_interval
        self.arm = dvrk.psm(ral = ral,
                            arm_name = arm_name,
                            expected_interval = expected_interval)
        self.arm.check_connections()

    # homing example
    def home(self):
        print('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print('move to starting position')
        goal = numpy.copy(self.arm.setpoint_jp())
        # go to zero position, make sure 3rd joint is past cannula
        goal.fill(0)
        goal[2] = 0.12
        self.arm.move_jp(goal).wait()

    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        goal = numpy.copy(self.arm.setpoint_jp())
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            # set in position joint mode
            goal[0] = 0.0
            goal[1] = 0.0
            goal[2] = 0.12
            self.arm.move_jp().wait()

    # effort jaw control example
    def jaw_effort(self):
        print('starting jaw effort')
        # get current joint torques just to set size
        effort_joint = numpy.copy(self.arm.measured_jf())
        print(effort_joint)
        effort_joint.fill(0.0)

        print('the jaws will open using a constant effort, the arm will go limp')
        input('press Enter to continue...')
        self.arm.servo_jf(effort_joint)
        self.arm.jaw.servo_jf(numpy.array([0.015]))

        print('the jaws will close using a constant effort, you can place a small object between the jaws now')
        input('press Enter to continue...')
        self.arm.jaw.servo_jf(numpy.array([-0.02]))

        print('the jaws will be released')
        input('press Enter to continue')
        self.arm.jaw.servo_jf(numpy.array([0.0]))

    # effort joint control example
    def joint_effort(self):
        print('starting joint effort')
        # get current joint torques just to set size
        effort_joint = numpy.copy(self.arm.measured_jf())
        effort_joint.fill(0.0)

        print('the jaws and arm will go limp')
        input('press Enter to continue...')
        self.arm.servo_jf(effort_joint)
        self.arm.jaw.servo_jf(numpy.array([0.0]))

        print('hold the arm from the top (near white clutch button) and keep an hand on the Enter key, the arm will push on the first joint')
        input('press Enter to continue...')
        effort_joint[0] = -0.5
        self.arm.servo_jf(effort_joint)

        print('arm will now push in opposite direction')
        input('press Enter to continue...')
        effort_joint[0] = 0.5
        self.arm.servo_jf(effort_joint)

        print('arm will now apply sine wave forces on first two joints')
        input('press Enter to continue...')
        duration = 10  # seconds
        samples = duration / self.expected_interval

        sleep_rate = self.ral.create_rate(1.0 / self.expected_interval)
        # create a new goal starting with current position
        for i in range(int(samples)):
            effort_joint[0] = 0.5 *  (1.0 - math.cos(i * 10.0 * math.radians(360.0) / samples))
            effort_joint[1] = 0.5 *  (1.0 - math.cos(i * 10.0 * math.radians(360.0) / samples))
            self.arm.servo_jf(effort_joint)
            sleep_rate.sleep()

        print('arm will now go limp')
        input('press Enter to continue...')
        effort_joint.fill(0.0)
        self.arm.servo_jf(effort_joint)

    # wrench and jaw effort control example
    def wrench_jaw_effort(self):
        print('starting wrench jaw effort')
        # get current joint torques just to set size
        effort_joint = numpy.copy(self.arm.measured_jf())
        effort_joint.fill(0.0)

        print('the jaws will open and arm will go limp')
        input('press Enter to continue...')
        self.arm.servo_jf(effort_joint)
        self.arm.jaw.servo_jf(numpy.array([0.015]))

        print('the jaws will close using a constant effort, you can place a small object between the jaws now')
        input('press Enter to continue...')
        self.arm.jaw.servo_jf(numpy.array([-0.02]))

        print('hold the arm close to the tool tip and keep an hand on the Enter key, the arm will push in Z direction')
        input('press Enter to continue...')
        self.arm.body_set_cf_orientation_absolute(True)
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0]))

        print('the jaws will be released')
        input('press Enter to continue...')
        self.arm.jaw.servo_jf(numpy.array([0.0]))

        print('arm will now go limp')
        input('press Enter to continue...')
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    # main method
    def run(self):
        self.home()
        self.jaw_effort()
        self.joint_effort()
        self.wrench_jaw_effort()


if __name__ == '__main__':
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_psm_effort_test')
    application = example_application(ral, args.arm, args.interval)
    ral.spin_and_execute(application.run)
