#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2015-02-22

# (C) Copyright 2015-2017 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_robot dvrk_arm_test.py <arm-name>

import dvrk
import math
import sys
import rospy
import numpy
import PyKDL

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name):
        print rospy.get_caller_id(), ' -> configuring dvrk_psm_test for ', robot_name
        self.arm = dvrk.psm(robot_name)

    # homing example
    def home(self):
        print rospy.get_caller_id(), ' -> starting home'
        self.arm.home()
        # get current joints just to set size
        goal = numpy.copy(self.arm.get_current_joint_position())
        # go to zero position
        goal.fill(0)
        self.arm.move_joint(goal, interpolate = True)

    # goal jaw control example
    def jaw_goal(self):
        print rospy.get_caller_id(), ' -> starting jaw goal'
        # try to open and close with the cartesian part of the arm in different modes
        print rospy.get_caller_id(), ' -> close and open without setting mode'

        print rospy.get_caller_id(), ' -> close and open after trajectory joint move, jaw should close/open twice'
        raw_input("Press Enter to continue...")
        initial_joint_position = numpy.copy(self.arm.get_current_joint_position())
        self.arm.move_joint(initial_joint_position, interpolate = True)
        self.arm.close_jaw()
        self.arm.open_jaw()
        self.arm.close_jaw()
        self.arm.open_jaw()

    # main method
    def run(self):
        self.home()
        self.jaw_goal()

if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print sys.argv[0], ' requires one argument, i.e. name of dVRK arm'
        else:
            application = example_application()
            application.configure(sys.argv[1])
            application.run()

    except rospy.ROSInterruptException:
        pass
