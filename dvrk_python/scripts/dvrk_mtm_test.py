#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2017-07-22

# (C) Copyright 2017-2019 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_arm_test.py <arm-name>

from __future__ import print_function
import dvrk
import sys
import rospy
import numpy
import threading
from sensor_msgs.msg import Joy


# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name):
        print(rospy.get_caller_id(), ' -> configuring dvrk_arm_test for ', robot_name)
        self.arm = dvrk.mtm(robot_name)
        self.coag_event = threading.Event()
        rospy.Subscriber('/dvrk/footpedals/coag',
                         Joy, self.coag_event_cb)

    # homing example
    def home(self):
        print(rospy.get_caller_id(), ' -> starting home')
        self.arm.home()
        # get current joints just to set size
        goal = numpy.copy(self.arm.get_current_joint_position())
        # go to zero position
        goal.fill(0)
        self.arm.move_joint(goal, interpolate=True)

    # foot pedal callback
    def coag_event_cb(self, data):
        if data.buttons[0] == 1:
            self.coag_event.set()

    # wait for foot pedal
    def wait_for_coag(self):
        self.coag_event.clear()
        self.coag_event.wait(60)

    # tests
    def tests(self):
        # turn on gravity compensation
        self.arm.set_gravity_compensation(True)

        print(rospy.get_caller_id(), ' -> press COAG pedal to move to next test (or just wait 60 seconds)')

        print(rospy.get_caller_id(), ' -> arm will go limp, hold it and press coag')
        self.wait_for_coag()
        self.arm.set_wrench_body_force((0.0, 0.0, 0.0))

        print(rospy.get_caller_id(), ' -> keep holding arm, press coag, a force in body frame will be applied')
        self.wait_for_coag()
        self.arm.set_wrench_body_orientation_absolute(False)
        self.arm.set_wrench_body_force((0.0, 0.0, -3.0))

        print(rospy.get_caller_id(), ' -> keep holding arm, press coag, a force in world frame will be applied')
        self.wait_for_coag()
        self.arm.set_wrench_body_orientation_absolute(True)
        self.arm.set_wrench_body_force((0.0, 0.0, -3.0))

        print(rospy.get_caller_id(), ' -> keep holding arm, press coag, orientation will be locked')
        self.wait_for_coag()
        self.arm.lock_orientation_as_is()

        print(rospy.get_caller_id(), ' -> keep holding arm, press coag, force will be removed')
        self.wait_for_coag()
        self.arm.set_wrench_body_force((0.0, 0.0, 0.0))

        print(rospy.get_caller_id(), ' -> keep holding arm, press coag, orientation will be unlocked')
        self.wait_for_coag()
        self.arm.unlock_orientation()

        print(rospy.get_caller_id(), ' -> keep holding arm, press coag, arm will freeze in position')
        self.wait_for_coag()
        self.arm.move(self.arm.get_desired_position())

        print(rospy.get_caller_id(), ' -> press coag to end')
        self.wait_for_coag()

    # main method
    def run(self):
        self.home()
        self.tests()


if __name__ == '__main__':
    try:
        if len(sys.argv) != 2:
            print(sys.argv[0], ' requires one argument, i.e. MTML or MTMR')
        else:
            application = example_application()
            application.configure(sys.argv[1])
            application.run()

    except rospy.ROSInterruptException:
        pass
