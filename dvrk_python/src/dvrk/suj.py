#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2018 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import rospy
import numpy
import PyKDL

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf_conversions import posemath

class suj(object):
    """Simple arm API wrapping around ROS messages
    """

    # initialize the arm
    def __init__(self, arm_name, ros_namespace = '/dvrk/SUJ/'):
        """Constructor.  This initializes a few data members.It
        requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `/dvrk/SUJ/PSM1`"""
        # data members, event based
        self.__arm_name = arm_name
        self.__ros_namespace = ros_namespace

        # continuous publish from dvrk_bridge
        self.__position_joint_current = numpy.array(0, dtype = numpy.float)
        self.__position_cartesian_current = PyKDL.Frame()
        self.__position_cartesian_local_current = PyKDL.Frame()

        # publishers
        self.__full_ros_namespace = self.__ros_namespace + self.__arm_name
        self.__set_position_joint_pub = rospy.Publisher(self.__full_ros_namespace
                                                        + '/set_position_joint',
                                                        JointState, latch = False, queue_size = 1)

        # subscribers
        rospy.Subscriber(self.__full_ros_namespace + '/state_joint_current',
                         JointState, self.__state_joint_current_cb),
        rospy.Subscriber(self.__full_ros_namespace + '/position_cartesian_current',
                         PoseStamped, self.__position_cartesian_current_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/position_cartesian_local_current',
                         PoseStamped, self.__position_cartesian_local_current_cb)

        # create node
        if not rospy.get_node_uri():
            rospy.init_node('suj_api', anonymous = True, log_level = rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')

    def __state_joint_current_cb(self, data):
        """Callback for the current joint position.

        :param data: the `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_current"""
        self.__position_joint_current.resize(len(data.position))
        self.__position_joint_current.flat[:] = data.position

    def __position_cartesian_current_cb(self, data):
        """Callback for the current cartesian position.

        :param data: The cartesian position current."""
        self.__position_cartesian_current = posemath.fromMsg(data.pose)

    def __position_cartesian_local_current_cb(self, data):
        """Callback for the current cartesian_local position.

        :param data: The cartesian_local position current."""
        self.__position_cartesian_local_current = posemath.fromMsg(data.pose)

    def get_current_joint_position(self):
        """Get the :ref:`current joint position <currentvdesired>` of
        the arm.

        :returns: the current position of the arm in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__position_joint_current

    def move_joint(self, end_joint):
        """Move the arm to the end vector, this only works with the SUJs in simulated mode.

        :param end_joint: the list of joints in which you should conclude movement
        # go to that position directly"""
        joint_state = JointState()
        joint_state.position[:] = end_joint.flat
        self.__set_position_joint_pub.publish(joint_state)

    def get_current_position(self):
        """Get the :ref:`current cartesian position <currentvdesired>` of the arm.

        :returns: the current position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__position_cartesian_current

    def get_current_position_local(self):
        """Get the :ref:`current cartesian position <currentvdesired>` of the arm.

        :returns: the current position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__position_cartesian_local_current
