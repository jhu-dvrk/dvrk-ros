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
# > rosrun dvrk_python dvrk_mtm_cartesian_impedance.py -a <arm-name>

import argparse
import crtk
from crtk_msgs.msg import CartesianImpedance
import dvrk
import numpy
import sys


class example_application:
    def __init__(self, ral, arm_name, expected_interval):
        print('configuring dvrk_mtm_cartesian_impedance for {}'.format(arm_name))
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

        gains = CartesianImpedance()
        # set orientation to identity quaternions
        gains.force_orientation.w = 1.0
        gains.torque_orientation.w = 1.0

        print('press and release the COAG pedal to move to next example, always hold the arm')

        print('arm will be constrained in X/Y plane around the current position')
        self.coag.wait(value = 0)
        # set gains in z direction
        gains.position_negative.p.z = -200.0
        gains.position_positive.p.z = -200.0
        gains.position_negative.d.z = -5.0
        gains.position_positive.d.z = -5.0
        gains.force_position.x = self.arm.measured_cp().p[0]
        gains.force_position.y = self.arm.measured_cp().p[1]
        gains.force_position.z = self.arm.measured_cp().p[2]
        self.arm.servo_ci(gains)

        print('orientation will be locked')
        self.coag.wait(value = 0)
        self.arm.lock_orientation_as_is()

        print('arm will be constrained in X/Y half plane around the current position')
        self.coag.wait(value = 0)
        # set gains in z direction, stiffer in half positive, 0 in negative
        gains.position_negative.p.z = 0.0
        gains.position_positive.p.z = -500.0
        gains.position_negative.d.z = 0.0
        gains.position_positive.d.z = -15.0
        gains.force_position.x = self.arm.measured_cp().p[0]
        gains.force_position.y = self.arm.measured_cp().p[1]
        gains.force_position.z = self.arm.measured_cp().p[2]
        self.arm.servo_ci(gains)

        print('an horizontal line will be created around the current position, with viscosity along the line')
        self.coag.wait(value = 0)
        # set gains in x, z directions for the line
        gains.position_negative.p.x = -200.0
        gains.position_positive.p.x = -200.0
        gains.position_negative.d.x = -5.0
        gains.position_positive.d.x = -5.0
        gains.position_negative.p.z = -200.0
        gains.position_positive.p.z = -200.0
        gains.position_negative.d.z = -5.0
        gains.position_positive.d.z = -5.0
        # viscosity along the line
        gains.position_negative.d.y = -10.0
        gains.position_positive.d.y = -10.0
        # always start from current position to avoid jumps
        gains.force_position.x = self.arm.measured_cp().p[0]
        gains.force_position.y = self.arm.measured_cp().p[1]
        gains.force_position.z = self.arm.measured_cp().p[2]
        self.arm.servo_ci(gains)

        print('a plane will be created perpendicular to the master gripper')
        self.coag.wait(value = 0)
        # set gains in x, z directions for the line
        gains.position_negative.p.x = 0.0
        gains.position_positive.p.x = 0.0
        gains.position_negative.d.x = 0.0
        gains.position_positive.d.x = 0.0
        gains.position_negative.p.y = 0.0
        gains.position_positive.p.y = 0.0
        gains.position_negative.d.y = 0.0
        gains.position_positive.d.y = 0.0
        gains.position_negative.p.z = -200.0
        gains.position_positive.p.z = -200.0
        gains.position_negative.d.z = -5.0
        gains.position_positive.d.z = -5.0

        stiffOri = -0.2
        dampOri = -0.01
        gains.orientation_negative.p.x = stiffOri
        gains.orientation_positive.p.x = stiffOri
        gains.orientation_negative.d.x = dampOri
        gains.orientation_positive.d.x = dampOri
        gains.orientation_negative.p.y = stiffOri
        gains.orientation_positive.p.y = stiffOri
        gains.orientation_negative.d.y = dampOri
        gains.orientation_positive.d.y = dampOri
        gains.orientation_negative.p.z = 0.0
        gains.orientation_positive.p.z = 0.0
        gains.orientation_negative.d.z = 0.0
        gains.orientation_positive.d.z = 0.0

        # always start from current position to avoid jumps
        gains.force_position.x = self.arm.measured_cp().p[0]
        gains.force_position.y = self.arm.measured_cp().p[1]
        gains.force_position.z = self.arm.measured_cp().p[2]
        orientationQuaternion = self.arm.measured_cp().M.GetQuaternion()
        gains.force_orientation.x = orientationQuaternion[0]
        gains.force_orientation.y = orientationQuaternion[1]
        gains.force_orientation.z = orientationQuaternion[2]
        gains.force_orientation.w = orientationQuaternion[3]
        gains.torque_orientation.x = orientationQuaternion[0]
        gains.torque_orientation.y = orientationQuaternion[1]
        gains.torque_orientation.z = orientationQuaternion[2]
        gains.torque_orientation.w = orientationQuaternion[3]
        self.arm.servo_ci(gains)
        self.arm.unlock_orientation()

        print('arm will freeze in position')
        self.coag.wait(value = 0)
        self.arm.hold()

    # main method
    def run(self):
        self.home()
        self.tests()


def main():
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['MTML', 'MTMR'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_mtm_cartesian_impedance')
    application = example_application(ral, args.arm, args.interval)
    ral.spin_and_execute(application.run)


if __name__ == '__main__':
    main()
