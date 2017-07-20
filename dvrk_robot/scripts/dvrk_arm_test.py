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

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name):
        print rospy.get_caller_id(), ' -> configuring dvrk_arm_test for ', robot_name
        self._arm = dvrk.arm(robot_name)

    # homing example
    def home(self):
        print rospy.get_caller_id(), ' -> starting home'
        self._arm.home()
        # get current joints just to set size
        goal = numpy.copy(self._arm.get_current_joint_position())
        # go to zero position
        goal.fill(0)
        self._arm.move_joint(goal, interpolate = True)

    # direct joint control example
    def joint_direct(self):
        print rospy.get_caller_id(), ' -> starting joint direct'
        # get current position
        initial_joint_position = numpy.copy(self._arm.get_current_joint_position())
        print rospy.get_caller_id(), ' -> testing direct joint position for 2 joints of ', len(initial_joint_position)
        amplitude = math.radians(10.0) # +/- 10 degrees
        duration = 5  # seconds
        rate = 200 # aiming for 200 Hz
        samples = duration * rate
        # create a new goal starting with current position
        goal = numpy.copy(initial_joint_position)
        for i in xrange(samples):
            goal[0] = initial_joint_position[0] + amplitude *  math.sin(i * math.radians(360.0) / samples)
            goal[1] = initial_joint_position[1] + amplitude *  math.sin(i * math.radians(360.0) / samples)
            self._arm.move_joint(goal, interpolate = False)
            rospy.sleep(1.0 / rate)
        print rospy.get_caller_id(), ' <- joint direct complete'

    # goal joint control example
    def joint_goal(self):
        print rospy.get_caller_id(), ' -> starting joint goal'
        # get current position
        initial_joint_position = numpy.copy(self._arm.get_current_joint_position())
        print rospy.get_caller_id(), ' -> testing goal joint position for 2 joints of ', len(initial_joint_position)
        amplitude = math.radians(10.0)
        # create a new goal starting with current position
        goal = numpy.copy(initial_joint_position)
        # first motion
        goal[0] = initial_joint_position[0] + amplitude
        goal[1] = initial_joint_position[1] - amplitude
        self._arm.move_joint(goal, interpolate = True)
        # second motion
        goal[0] = initial_joint_position[0] - amplitude
        goal[1] = initial_joint_position[1] + amplitude
        self._arm.move_joint(goal, interpolate = True)
        # back to initial position
        self._arm.move_joint(initial_joint_position, interpolate = True)
        print rospy.get_caller_id(), ' <- joint goal complete'

    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        initial_joint_position = self._position_joint_desired
        if ((self._robot_name == 'PSM1') or (self._robot_name == 'PSM2') or (self._robot_name == 'PSM3') or (self._robot_name == 'ECM')):
            # set in position joint mode
            self.set_state_block(state = 'DVRK_POSITION_GOAL_JOINT')
                # create a new goal starting with current position
            goal = numpy.array(0, dtype = numpy.float)
            goal.position[:] = initial_joint_position
            goal.position[0] = 0.0
            goal.position[1] = 0.0
            goal.position[2] = 0.12
            self._goal_reached_event.clear()
            self.set_position_goal_joint.publish(goal)
            self._goal_reached_event.wait(60) # 1 minute at most
            if not self._goal_reached:
                rospy.signal_shutdown('failed to reach goal')
                sys.exit(-1)

    # direct cartesian control example
    def cartesian_direct(self):
        print rospy.get_caller_id(), ' -> starting cartesian direct'
        self.prepare_cartesian()
        # set in position cartesian mode
        self.set_state_block('DVRK_POSITION_CARTESIAN')
        # get current position
        initial_cartesian_position = self._position_cartesian_desired
        goal = Pose()
        # create a new goal starting with current position
        goal.position.x = initial_cartesian_position.position.x
        goal.position.y = initial_cartesian_position.position.y
        goal.position.z = initial_cartesian_position.position.z
        goal.orientation.x = initial_cartesian_position.orientation.x
        goal.orientation.y = initial_cartesian_position.orientation.y
        goal.orientation.z = initial_cartesian_position.orientation.z
        goal.orientation.w = initial_cartesian_position.orientation.w
        # motion parameters
        amplitude = 0.05 # 5 cm
        duration = 5  # 5 seconds
        rate = 200 # aiming for 200 Hz
        samples = duration * rate
        for i in xrange(samples):
            goal.position.x =  initial_cartesian_position.position.x + amplitude *  math.sin(i * math.radians(360.0) / samples)
            goal.position.y =  initial_cartesian_position.position.y + amplitude *  math.sin(i * math.radians(360.0) / samples)
            self.set_position_cartesian.publish(goal)
            errorX = goal.position.x - self._position_cartesian_desired.position.x
            errorY = goal.position.y - self._position_cartesian_desired.position.y
            errorZ = goal.position.z - self._position_cartesian_desired.position.z
            error = math.sqrt(errorX * errorX + errorY * errorY + errorZ * errorZ)
            if error > 0.002: # 2 mm
                print 'Inverse kinematic error in position [', i, ']: ', error
            rospy.sleep(1.0 / rate)
        print rospy.get_caller_id(), ' <- cartesian direct complete'

    # wrapper around publisher/subscriber to manage events
    def set_position_goal_cartesian_publish_and_wait(self, goal):
        self._goal_reached_event.clear()
        self._goal_reached = False
        self.set_position_goal_cartesian.publish(goal)
        self._goal_reached_event.wait(60) # 1 minute at most
        if not self._goal_reached:
            rospy.signal_shutdown('failed to reach goal')
            sys.exit(-1)

    # direct cartesian control example
    def cartesian_goal(self):
        print rospy.get_caller_id(), ' -> starting cartesian goal'
        self.prepare_cartesian()
        # set in position cartesian mode
        self.set_state_block('DVRK_POSITION_GOAL_CARTESIAN')
        # get current position
        initial_cartesian_position = self._position_cartesian_desired
        goal = Pose()
        # create a new goal starting with current position
        goal.position.x = initial_cartesian_position.position.x
        goal.position.y = initial_cartesian_position.position.y
        goal.position.z = initial_cartesian_position.position.z
        goal.orientation.x = initial_cartesian_position.orientation.x
        goal.orientation.y = initial_cartesian_position.orientation.y
        goal.orientation.z = initial_cartesian_position.orientation.z
        goal.orientation.w = initial_cartesian_position.orientation.w
        # motion parameters
        amplitude = 0.05 # 5 cm
        # first motion
        goal.position.x =  initial_cartesian_position.position.x - amplitude
        goal.position.y =  initial_cartesian_position.position.y
        self.set_position_goal_cartesian_publish_and_wait(goal)
        # second motion
        goal.position.x =  initial_cartesian_position.position.x + amplitude
        goal.position.y =  initial_cartesian_position.position.y
        self.set_position_goal_cartesian_publish_and_wait(goal)
        # back to initial position
        goal.position.x =  initial_cartesian_position.position.x
        goal.position.y =  initial_cartesian_position.position.y
        self.set_position_goal_cartesian_publish_and_wait(goal)
        # first motion
        goal.position.x =  initial_cartesian_position.position.x
        goal.position.y =  initial_cartesian_position.position.y - amplitude
        self.set_position_goal_cartesian_publish_and_wait(goal)
        # second motion
        goal.position.x =  initial_cartesian_position.position.x
        goal.position.y =  initial_cartesian_position.position.y + amplitude
        self.set_position_goal_cartesian_publish_and_wait(goal)
        # back to initial position
        goal.position.x =  initial_cartesian_position.position.x
        goal.position.y =  initial_cartesian_position.position.y
        self.set_position_goal_cartesian_publish_and_wait(goal)
        print rospy.get_caller_id(), ' <- cartesian goal complete'

    # main method
    def run(self):
        self.home()
        self.joint_direct()
        self.joint_goal()
        # self.cartesian_direct()
        # self.cartesian_goal()

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
