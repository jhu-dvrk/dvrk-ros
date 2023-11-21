#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2017-07-22

# (C) Copyright 2017-2023 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>
# Run test script:
# > rosrun dvrk_python dvrk_mtm_test.py -a <arm-name>

import argparse
import crtk
import dvrk
import numpy
import sys


class example_application:
    def __init__(self, ral, arm_name, expected_interval):
        print('configuring dvrk_mtm_test for {}'.format(arm_name))
        self.ral = ral
        self.expected_interval = expected_interval
        self.arm = dvrk.mtm(ral = ral,
                            arm_name = arm_name,
                            expected_interval = expected_interval)
        self.coag = crtk.joystick_button(ral, 'footpedals/coag', 0)

    # homing example
    def home(self):
        self.ral.check_connections()

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
        self.arm.move_jp(goal).wait()

    # tests
    def tests(self):
        # turn on gravity compensation
        self.arm.use_gravity_compensation(True)

        print('press and release the COAG pedal to move to the next example, always hold the arm')
        print('arm will go limp')
        self.coag.wait(value = 0)
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        print('a force in body frame will be applied (direction depends on wrist orientation)')
        self.coag.wait(value = 0)
        self.arm.body_set_cf_orientation_absolute(False)
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, -3.0, 0.0, 0.0, 0.0]))

        print('a force in world frame will be applied (fixed direction)')
        self.coag.wait(value = 0)
        self.arm.body_set_cf_orientation_absolute(True)
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, -3.0, 0.0, 0.0, 0.0]))

        print('orientation will be locked')
        self.coag.wait(value = 0)
        self.arm.lock_orientation_as_is()

        print('force will be removed')
        self.coag.wait(value = 0)
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        print('orientation will be unlocked')
        self.coag.wait(value = 0)
        self.arm.unlock_orientation()

        print('arm will freeze in position')
        self.coag.wait(value = 0)
        self.arm.hold()

        print('press and release coag to end')
        self.coag.wait(value = 0)

    # main method
    def run(self):
        self.home()
        self.tests()


def main():
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name
    ral = crtk.ral('dvrk_mtm_test')

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['MTML', 'MTMR'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv)

    application = example_application(ral, args.arm, args.interval)
    ral.spin_and_execute(application.run)

if __name__ == '__main__':
    main()
