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
# > rosrun dvrk_python dvrk_mtm_cartesian_impedance <arm-name>

from __future__ import print_function
import dvrk
import sys
import rospy
import numpy
import threading
from sensor_msgs.msg import Joy
from cisst_msgs.msg import prmCartesianImpedanceGains

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name):
        print(rospy.get_caller_id(), ' -> configuring dvrk_arm_test for ', robot_name)
        self.arm = dvrk.mtm(robot_name)
        self.coag_event = threading.Event()
        rospy.Subscriber('/dvrk/footpedals/coag',
                         Joy, self.coag_event_cb)
        self.set_gains_pub = rospy.Publisher(self.arm._arm__full_ros_namespace + '/set_cartesian_impedance_gains',
                                             prmCartesianImpedanceGains, latch = True, queue_size = 1)

    # homing example
    def home(self):
        print(rospy.get_caller_id(), ' -> starting home')
        self.arm.home()
        # get current joints just to set size
        goal = numpy.copy(self.arm.get_current_joint_position())
        # go to zero position
        goal.fill(0)
        self.arm.move_joint(goal, interpolate = True)

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
        self.arm.set_gravity_compensation(True)

        gains = prmCartesianImpedanceGains()
        # set orientation to identity quaternions
        gains.ForceOrientation.w = 1.0
        gains.TorqueOrientation.w = 1.0

        print(rospy.get_caller_id(), ' -> press COAG pedal to move to next example')

        print(rospy.get_caller_id(), ' -> arm will be constrained in X/Y plane around the current position')
        self.wait_for_coag()
        # set gains in z direction
        gains.PosStiffNeg.z = -200.0
        gains.PosStiffPos.z = -200.0
        gains.PosDampingNeg.z = -5.0
        gains.PosDampingPos.z = -5.0
        gains.ForcePosition.x = self.arm.get_current_position().p[0]
        gains.ForcePosition.y = self.arm.get_current_position().p[1]
        gains.ForcePosition.z = self.arm.get_current_position().p[2]
        self.set_gains_pub.publish(gains)

        print(rospy.get_caller_id(), ' -> orientation will be locked')
        self.wait_for_coag()
        self.arm.lock_orientation_as_is()

        print(rospy.get_caller_id(), ' -> arm will be constrained in X/Y half plane around the current position')
        self.wait_for_coag()
        # set gains in z direction, stiffer in half positive, 0 in negative
        gains.PosStiffNeg.z = 0.0
        gains.PosStiffPos.z = -500.0
        gains.PosDampingNeg.z = 0.0
        gains.PosDampingPos.z = -15.0
        gains.ForcePosition.x = self.arm.get_current_position().p[0]
        gains.ForcePosition.y = self.arm.get_current_position().p[1]
        gains.ForcePosition.z = self.arm.get_current_position().p[2]
        self.set_gains_pub.publish(gains)

        print(rospy.get_caller_id(), ' -> an horizontal line will be created around the current position, with viscosity along the line')
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
        gains.ForcePosition.x = self.arm.get_current_position().p[0]
        gains.ForcePosition.y = self.arm.get_current_position().p[1]
        gains.ForcePosition.z = self.arm.get_current_position().p[2]
        self.set_gains_pub.publish(gains)

        print(rospy.get_caller_id(), ' -> a plane will be created perpendicular to the master gripper')
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
        gains.ForcePosition.x = self.arm.get_current_position().p[0]
        gains.ForcePosition.y = self.arm.get_current_position().p[1]
        gains.ForcePosition.z = self.arm.get_current_position().p[2]
        orientationQuaternion = self.arm.get_current_position().M.GetQuaternion()
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
        if (len(sys.argv) != 2):
            print(sys.argv[0], ' requires one argument, i.e. MTML or MTMR')
        else:
            application = example_application()
            application.configure(sys.argv[1])
            application.run()

    except rospy.ROSInterruptException:
        pass
