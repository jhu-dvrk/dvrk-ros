#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2017-07-22

# (C) Copyright 2017-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
import sys
import rospy
import numpy
import threading
import argparse
from sensor_msgs.msg import Joy

# print with node id
def print_id(message):
    print('%s -> %s' % (rospy.get_caller_id(), message))

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name, expected_interval):
        print_id('configuring dvrk_mtm_test for %s' % robot_name)
        self.expected_interval = expected_interval
        self.arm = dvrk.mtm(arm_name = robot_name,
                            expected_interval = expected_interval)
        self.coag_event = threading.Event()
        rospy.Subscriber('footpedals/coag',
                         Joy, self.coag_event_cb)

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
        # go to zero position, make sure 3rd joint is past cannula
        goal.fill(0)
        self.arm.move_jp(goal).wait()

    # foot pedal callback
    def coag_event_cb(self, data):
        if data.buttons[0] == 1:
            self.coag_event.set()

    # wait for foot pedal
    def wait_for_coag(self):
        self.coag_event.clear()
        self.coag_event.wait(100000)

    # tests
    def tests(self):
        # turn on gravity compensation
        self.arm.use_gravity_compensation(True)

        print_id('press COAG pedal to move to the next test')

        print_id('arm will go limp, hold it and press coag')
        self.wait_for_coag()
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        print_id('keep holding arm, press coag, a force in body frame will be applied (direction depends on wrist orientation)')
        self.wait_for_coag()
        self.arm.body_set_cf_orientation_absolute(False)
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, -3.0, 0.0, 0.0, 0.0]))

        print_id('keep holding arm, press coag, a force in world frame will be applied (fixed direction)')
        self.wait_for_coag()
        self.arm.body_set_cf_orientation_absolute(True)
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, -3.0, 0.0, 0.0, 0.0]))

        print_id('keep holding arm, press coag, orientation will be locked')
        self.wait_for_coag()
        self.arm.lock_orientation_as_is()

        print_id('keep holding arm, press coag, force will be removed')
        self.wait_for_coag()
        self.arm.body.servo_cf(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        print_id('keep holding arm, press coag, orientation will be unlocked')
        self.wait_for_coag()
        self.arm.unlock_orientation()

        print_id('keep holding arm, press coag, arm will freeze in position')
        self.wait_for_coag()
        self.arm.move_jp(self.arm.measured_jp()).wait()

        print_id('press coag to end')
        self.wait_for_coag()

    # main method
    def run(self):
        self.home()
        self.tests()


if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('dvrk_mtm_test')
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['MTML', 'MTMR'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    application = example_application()
    application.configure(args.arm, args.interval)
    application.run()
