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
# > rosrun dvrk_python dvrk_arm_test.py <arm-name>

from __future__ import print_function
import dvrk
import math
import sys
import rospy
import numpy
import PyKDL


if sys.version_info.major < 3:
    input = raw_input


# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name):
        print(rospy.get_caller_id(), ' -> configuring dvrk_psm_test for ', robot_name)
        self.arm = dvrk.psm(robot_name)

    # homing example
    def home(self):
        print(rospy.get_caller_id(), ' -> starting home')
        self.arm.home()
        # get current joints just to set size
        goal = numpy.copy(self.arm.get_current_joint_position())
        # go to zero position, except for insert joint so we can't break tool in cannula
        goal.fill(0)
        goal[2] = 0.12
        self.arm.move_joint(goal, interpolate = True)

    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        goal = numpy.copy(self.arm.get_current_joint_position())
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2') or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            # set in position joint mode
            goal[0] = 0.0
            goal[1] = 0.0
            goal[2] = 0.12
            self.arm.move_joint(goal, interpolate = True)

    # goal jaw control example
    def jaw_goal(self):
        print(rospy.get_caller_id(), ' -> starting jaw goal')
        # try to open and close with the cartesian part of the arm in different modes
        print(rospy.get_caller_id(), '   -> close and open without other move command')
        input("    Press Enter to continue...")
        self.arm.home()
        self.arm.close_jaw()
        self.arm.open_jaw()
        self.arm.close_jaw()
        self.arm.open_jaw()
        # try to open and close with a joint goal
        print(rospy.get_caller_id(), '   -> close and open with joint move command')
        input("    Press Enter to continue...")
        self.arm.home()
        self.arm.close_jaw(blocking = False)
        self.arm.insert_tool(0.1)
        self.arm.open_jaw(blocking = False)
        self.arm.insert_tool(0.15)
        self.arm.close_jaw()
        self.arm.insert_tool(0.1)
        self.arm.open_jaw()
        self.arm.insert_tool(0.15)

        print(rospy.get_caller_id(), '   -> close and open with cartesian move command')
        input("    Press Enter to continue...")

        # try to open and close with a cartesian goal
        self.prepare_cartesian()

        initial_cartesian_position = PyKDL.Frame()
        initial_cartesian_position.p = self.arm.get_desired_position().p
        initial_cartesian_position.M = self.arm.get_desired_position().M
        goal = PyKDL.Frame()
        goal.p = self.arm.get_desired_position().p
        goal.M = self.arm.get_desired_position().M

        # motion parameters
        amplitude = 0.05 # 5 cm

        # first motion
        goal.p[0] =  initial_cartesian_position.p[0] - amplitude
        goal.p[1] =  initial_cartesian_position.p[1]
        self.arm.move(goal, blocking = False)
        self.arm.close_jaw()

        # second motion
        goal.p[0] =  initial_cartesian_position.p[0] + amplitude
        goal.p[1] =  initial_cartesian_position.p[1]
        self.arm.move(goal, blocking = False)
        self.arm.open_jaw()


    # goal jaw control example
    def jaw_direct(self):
        print(rospy.get_caller_id(), ' -> starting jaw direct')
        # try to open and close directly, needs interpolation
        print(rospy.get_caller_id(), '   -> close and open without other move command')
        input("    Press Enter to continue...")
        self.arm.move_jaw(math.radians(30.0))
        # assume we start at 30 the move +/- 30
        amplitude = math.radians(30.0)
        duration = 5  # seconds
        rate = 200 # aiming for 200 Hz
        samples = duration * rate
        # create a new goal starting with current position
        for i in range(samples):
            goal = math.radians(30.0) + amplitude * math.sin(i * math.radians(360.0) / samples)
            self.arm.move_jaw(goal, interpolate = False)
            rospy.sleep(1.0 / rate)


    # main method
    def run(self):
        self.home()
        self.jaw_goal()
        self.jaw_direct()
        self.jaw_goal() # just to make sure we can transition back to trajectory mode


if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print(sys.argv[0], ' requires one argument, i.e. name of dVRK arm')
        else:
            application = example_application()
            application.configure(sys.argv[1])
            application.run()

    except rospy.ROSInterruptException:
        pass
