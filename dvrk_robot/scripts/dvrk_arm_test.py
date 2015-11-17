#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2015-02-22

# Start a single arm using
# > rosrun dvrk_robot dvrk_mtm_ros   // or dvrk_psm_ros or dvrk_mtm_ros

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_robot dvrk_arm_test.py

import rospy
import threading
import math
import sys

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

# example of application with callbacks for robot events
class example_application:

    def __init__(self):
        # data members, event based
        self._robot_name = 'undefined'
        self._robot_state = 'uninitialized'
        self._robot_state_event = threading.Event()
        self._goal_reached = False
        self._goal_reached_event = threading.Event()

        # continuous publish from dvrk_bridge
        self._position_joint_desired = []
        self._position_cartesian_desired = Pose()

    # callbacks
    def robot_state_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " -> current state is %s", data.data)
        self._robot_state = data.data
        self._robot_state_event.set()

    def goal_reached_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " -> goal reached is %s", data.data)
        self._goal_reached = data.data
        self._goal_reached_event.set()

    def position_joint_desired_callback(self, data):
        self._position_joint_desired[:] = data.position

    def position_cartesian_desired_callback(self, data):
        self._position_cartesian_desired = data

    # configuration
    def configure(self, robot_name):
        self._robot_name = robot_name
        # publishers
        ros_namespace = '/dvrk/' + self._robot_name
        self.set_robot_state = rospy.Publisher(ros_namespace + '/set_robot_state', String, latch=True)
        self.set_position_joint = rospy.Publisher(ros_namespace + '/set_position_joint', JointState, latch=True)
        self.set_position_goal_joint = rospy.Publisher(ros_namespace + '/set_position_goal_joint', JointState, latch=True)
        self.set_position_cartesian = rospy.Publisher(ros_namespace + '/set_position_cartesian', Pose, latch=True)
        self.set_position_goal_cartesian = rospy.Publisher(ros_namespace + '/set_position_goal_cartesian', Pose, latch=True)
        # subscribers
        rospy.Subscriber(ros_namespace + '/robot_state', String, self.robot_state_callback)
        rospy.Subscriber(ros_namespace + '/goal_reached', Bool, self.goal_reached_callback)
        rospy.Subscriber(ros_namespace + '/position_joint_desired', JointState, self.position_joint_desired_callback)
        rospy.Subscriber(ros_namespace + '/position_cartesian_desired', Pose, self.position_cartesian_desired_callback)
        # create node
        rospy.init_node('dvrk_arm_test', anonymous=True)
        rospy.loginfo(rospy.get_caller_id() + ' -> started dvrk_arm_test')

    # simple set state with block
    def set_state_block(self, state, timeout = 60):
        self._robot_state_event.clear()
        self.set_robot_state.publish(state)
        self._robot_state_event.wait(timeout)
        if (self._robot_state != state):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state ' + state)
            rospy.signal_shutdown('failed to reach desired state')
            sys.exit(-1)

    # homing example
    def home(self):
        rospy.loginfo(rospy.get_caller_id() + ' -> requesting homing')
        self._robot_state_event.clear()
        self.set_robot_state.publish('Home')
        counter = 10 # up to 10 transitions to get ready
        while (counter > 0):
            self._robot_state_event.wait(60) # give up to 1 minute for each transition
            if (self._robot_state != 'DVRK_READY'):
                self._robot_state_event.clear()
                counter = counter - 1
                rospy.loginfo(rospy.get_caller_id() + ' -> waiting for state to be DVRK_READY')
            else:
                counter = -1
        if (self._robot_state != 'DVRK_READY'):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state DVRK_READY')
            rospy.signal_shutdown('failed to reach state DVRK_READY')
            sys.exit(-1)
        rospy.loginfo(rospy.get_caller_id() + ' <- homing complete')

    # direct joint control example
    def joint_direct(self):
        rospy.loginfo(rospy.get_caller_id() + ' -> starting joint direct')
        # set in position joint mode
        self.set_state_block('DVRK_POSITION_JOINT')
        # get current position
        initial_joint_position = []
        initial_joint_position[:] = self._position_joint_desired
        rospy.loginfo(rospy.get_caller_id() + " -> testing direct joint position for 2 joints of %i", len(initial_joint_position))
        amplitude = math.radians(10.0) # +/- 10 degrees
        duration = 5  # seconds
        rate = 200 # aiming for 200 Hz
        samples = duration * rate
        # create a new goal starting with current position
        goal = JointState()
        goal.position[:] = initial_joint_position
        for i in xrange(samples):
            goal.position[0] = initial_joint_position[0] + amplitude *  math.sin(i * math.radians(360.0) / samples)
            goal.position[1] = initial_joint_position[1] + amplitude *  math.sin(i * math.radians(360.0) / samples)
            self.set_position_joint.publish(goal)
            rospy.sleep(1.0 / rate)
        rospy.loginfo(rospy.get_caller_id() + ' <- joint direct complete')

    # wrapper around publisher/subscriber to manage events
    def set_position_goal_joint_publish_and_wait(self, goal):
        self._goal_reached_event.clear()
        self._goal_reached = False
        self.set_position_goal_joint.publish(goal)
        self._goal_reached_event.wait(60) # 1 minute at most
        if not self._goal_reached:
            rospy.signal_shutdown('failed to reach goal')
            sys.exit(-1)

    # goal joint control example
    def joint_goal(self):
        rospy.loginfo(rospy.get_caller_id() + ' -> starting joint goal')
        # set in position joint mode
        self.set_state_block('DVRK_POSITION_GOAL_JOINT')
        # get current position
        initial_joint_position = []
        initial_joint_position[:] = self._position_joint_desired
        rospy.loginfo(rospy.get_caller_id() + " -> testing goal joint position for 2 joints of %i", len(initial_joint_position))
        amplitude = math.radians(10.0)
        # create a new goal starting with current position
        goal = JointState()
        goal.position[:] = initial_joint_position
        # first motion
        goal.position[0] = initial_joint_position[0] + amplitude
        goal.position[1] = initial_joint_position[1] - amplitude
        self.set_position_goal_joint_publish_and_wait(goal)
        # second motion
        goal.position[0] = initial_joint_position[0] - amplitude
        goal.position[1] = initial_joint_position[1] + amplitude
        self.set_position_goal_joint_publish_and_wait(goal)
        # back to initial position
        goal.position[:] = initial_joint_position
        self.set_position_goal_joint_publish_and_wait(goal)
        rospy.loginfo(rospy.get_caller_id() + ' <- joint goal complete')

    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        initial_joint_position = self._position_joint_desired
        if ((self._robot_name == 'PSM1') or (self._robot_name == 'PSM2') or (self._robot_name == 'PSM3') or (self._robot_name == 'ECM')):
            # set in position joint mode
            self.set_state_block(state = 'DVRK_POSITION_GOAL_JOINT')
                # create a new goal starting with current position
            goal = JointState()
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
        rospy.loginfo(rospy.get_caller_id() + ' -> starting cartesian direct')
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
        rospy.loginfo(rospy.get_caller_id() + ' <- cartesian direct complete')

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
        rospy.loginfo(rospy.get_caller_id() + ' -> starting cartesian goal')
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
        rospy.loginfo(rospy.get_caller_id() + ' <- cartesian goal complete')

    # main method
    def run(self):
        # self.home()
        self.joint_direct()
        self.joint_goal()
        self.cartesian_direct()
        self.cartesian_goal()

if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
        else:
            application = example_application()
            application.configure(sys.argv[1])
            application.run()

    except rospy.ROSInterruptException:
        pass
