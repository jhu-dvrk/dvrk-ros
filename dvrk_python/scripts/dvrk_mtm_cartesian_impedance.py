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
# > rosrun dvrk_python dvrk_mtm_cartesian_impedance <arm-name>

import dvrk
import sys
import rospy
import numpy
import threading
import argparse
from sensor_msgs.msg import Joy
from cisst_msgs.msg import prmCartesianImpedanceGains

# print with node id
def print_id(message):
    print('%s -> %s' % (rospy.get_caller_id(), message))

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name, expected_interval):
        print_id('configuring dvrk_mtm_cartesian_impedance for %s' % robot_name)
        self.expected_interval = expected_interval
        self.arm = dvrk.mtm(arm_name = robot_name,
                            expected_interval = expected_interval)
        self.coag_event = threading.Event()
        rospy.Subscriber('footpedals/coag',
                         Joy, self.coag_event_cb)
        self.set_gains_pub = rospy.Publisher(self.arm._arm__full_ros_namespace + '/set_cartesian_impedance_gains',
                                             prmCartesianImpedanceGains, latch = True, queue_size = 1)

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
        if (data.buttons[0] == 1):
            self.coag_event.set()

    # wait for foot pedal
    def wait_for_coag(self):
        self.coag_event.clear()
        self.coag_event.wait(600)


    # tests
    def tests(self):
        # turn on gravity compensation
        self.arm.use_gravity_compensation(True)

        gains = prmCartesianImpedanceGains()
        # set orientation to identity quaternions
        gains.ForceOrientation.w = 1.0
        gains.TorqueOrientation.w = 1.0

        print_id('press COAG pedal to move to next example')

        print_id('arm will be constrained in X/Y plane around the current position')
        self.wait_for_coag()
        # set gains in z direction
        gains.PosStiffNeg.z = -200.0
        gains.PosStiffPos.z = -200.0
        gains.PosDampingNeg.z = -5.0
        gains.PosDampingPos.z = -5.0
        gains.ForcePosition.x = self.arm.measured_cp().p[0]
        gains.ForcePosition.y = self.arm.measured_cp().p[1]
        gains.ForcePosition.z = self.arm.measured_cp().p[2]
        self.set_gains_pub.publish(gains)

        print_id('orientation will be locked')
        self.wait_for_coag()
        self.arm.lock_orientation_as_is()

        print_id('arm will be constrained in X/Y half plane around the current position')
        self.wait_for_coag()
        # set gains in z direction, stiffer in half positive, 0 in negative
        gains.PosStiffNeg.z = 0.0
        gains.PosStiffPos.z = -500.0
        gains.PosDampingNeg.z = 0.0
        gains.PosDampingPos.z = -15.0
        gains.ForcePosition.x = self.arm.measured_cp().p[0]
        gains.ForcePosition.y = self.arm.measured_cp().p[1]
        gains.ForcePosition.z = self.arm.measured_cp().p[2]
        self.set_gains_pub.publish(gains)

        print_id('an horizontal line will be created around the current position, with viscosity along the line')
        self.wait_for_coag()
        # set gains in x, z directions for the line
        gains.PosStiffNeg.x = -200.0
        gains.PosStiffPos.x = -200.0
        gains.PosDampingNeg.x = -5.0
        gains.PosDampingPos.x = -5.0
        gains.PosStiffNeg.z = -200.0
        gains.PosStiffPos.z = -200.0
        gains.PosDampingNeg.z = -5.0
        gains.PosDampingPos.z = -5.0
        # viscosity along the line
        gains.PosDampingNeg.y = -10.0
        gains.PosDampingPos.y = -10.0
        # always start from current position to avoid jumps
        gains.ForcePosition.x = self.arm.measured_cp().p[0]
        gains.ForcePosition.y = self.arm.measured_cp().p[1]
        gains.ForcePosition.z = self.arm.measured_cp().p[2]
        self.set_gains_pub.publish(gains)

        print_id('a plane will be created perpendicular to the master gripper')
        self.wait_for_coag()
        # set gains in x, z directions for the line
        gains.PosStiffNeg.x = 0.0
        gains.PosStiffPos.x = 0.0
        gains.PosDampingNeg.x = 0.0
        gains.PosDampingPos.x = 0.0
        gains.PosStiffNeg.y = 0.0
        gains.PosStiffPos.y = 0.0
        gains.PosDampingNeg.y = 0.0
        gains.PosDampingPos.y = 0.0
        gains.PosStiffNeg.z = -200.0
        gains.PosStiffPos.z = -200.0
        gains.PosDampingNeg.z = -5.0
        gains.PosDampingPos.z = -5.0

        stiffOri = -0.2
        dampOri = -0.01
        gains.OriStiffNeg.x = stiffOri
        gains.OriStiffPos.x = stiffOri
        gains.OriDampingNeg.x = dampOri
        gains.OriDampingPos.x = dampOri
        gains.OriStiffNeg.y = stiffOri
        gains.OriStiffPos.y = stiffOri
        gains.OriDampingNeg.y = dampOri
        gains.OriDampingPos.y = dampOri
        gains.OriStiffNeg.z = 0.0
        gains.OriStiffPos.z = 0.0
        gains.OriDampingNeg.z = 0.0
        gains.OriDampingPos.z = 0.0

        # always start from current position to avoid jumps
        gains.ForcePosition.x = self.arm.measured_cp().p[0]
        gains.ForcePosition.y = self.arm.measured_cp().p[1]
        gains.ForcePosition.z = self.arm.measured_cp().p[2]
        orientationQuaternion = self.arm.measured_cp().M.GetQuaternion()
        gains.ForceOrientation.x = orientationQuaternion[0]
        gains.ForceOrientation.y = orientationQuaternion[1]
        gains.ForceOrientation.z = orientationQuaternion[2]
        gains.ForceOrientation.w = orientationQuaternion[3]
        gains.TorqueOrientation.x = orientationQuaternion[0]
        gains.TorqueOrientation.y = orientationQuaternion[1]
        gains.TorqueOrientation.z = orientationQuaternion[2]
        gains.TorqueOrientation.w = orientationQuaternion[3]
        self.set_gains_pub.publish(gains)
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
    rospy.init_node('dvrk_mtm_cartesian_impedance')
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
