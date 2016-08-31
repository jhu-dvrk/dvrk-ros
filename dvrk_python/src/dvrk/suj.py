#  Author(s):  Anton Deguet
#  Created on: 2016-05

#   (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import threading

import rospy
import PyKDL

from geometry_msgs.msg import PoseStamped
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
        self.__position_cartesian_desired = PyKDL.Frame()
        self.__position_cartesian_current = PyKDL.Frame()
        self.__position_cartesian_local_desired = PyKDL.Frame()
        self.__position_cartesian_local_current = PyKDL.Frame()

        # publishers
        self.__full_ros_namespace = self.__ros_namespace + self.__arm_name

        # subscribers
        rospy.Subscriber(self.__full_ros_namespace + '/position_cartesian_desired',
                         PoseStamped, self.__position_cartesian_desired_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/position_cartesian_current',
                         PoseStamped, self.__position_cartesian_current_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/position_cartesian_local_desired',
                         PoseStamped, self.__position_cartesian_local_desired_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/position_cartesian_local_current',
                         PoseStamped, self.__position_cartesian_local_current_cb)

        # create node
        rospy.init_node('arm_api', anonymous = True, log_level = rospy.WARN)

    def __position_cartesian_desired_cb(self, data):
        """Callback for the cartesian desired position.

        :param data: the cartesian position desired"""
        self.__position_cartesian_desired = posemath.fromMsg(data.pose)

    def __position_cartesian_current_cb(self, data):
        """Callback for the current cartesian position.

        :param data: The cartesian position current."""
        self.__position_cartesian_current = posemath.fromMsg(data.pose)

    def __position_cartesian_local_desired_cb(self, data):
        """Callback for the cartesian_local desired position.

        :param data: the cartesian_local position desired"""
        self.__position_cartesian_local_desired = posemath.fromMsg(data.pose)

    def __position_cartesian_local_current_cb(self, data):
        """Callback for the current cartesian_local position.

        :param data: The cartesian_local position current."""
        self.__position_cartesian_local_current = posemath.fromMsg(data.pose)

    def get_current_position(self):
        """Get the :ref:`current cartesian position <currentvdesired>` of the arm.

        :returns: the current position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__position_cartesian_current

    def get_desired_position(self):
        """Get the :ref:`desired cartesian position <currentvdesired>` of the arm.

        :returns: the desired position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__position_cartesian_desired

    def get_current_position_local(self):
        """Get the :ref:`current cartesian position <currentvdesired>` of the arm.

        :returns: the current position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__position_cartesian_local_current

    def get_desired_position_local(self):
        """Get the :ref:`desired cartesian position <currentvdesired>` of the arm.

        :returns: the desired position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__position_cartesian_local_desired
