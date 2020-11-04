#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2020 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

"""This class presents a arm api for the da Vinci Research Kit.
Remember that for this program to work, you will need to import the
arm class, this can be done by `from dvrk.arm import arm` as well as
initialize the arm. For example, if we want to create a arm called
`r`, for arm `PSM1`, we will simply type `r = arm('PSM1')`.

For arm specific features, import the class psm or mtm (e.g. `from
dvrk.psm import psm`) and initialize your instance using `psm1 =
psm('PSM1')`.
"""

# sphinx-apidoc -F -A "Yijun Hu" -o doc src
import threading
import math

import crtk
import rospy
import numpy
import PyKDL

# we should probably not import the symbols and put them in current namespace
from tf import transformations
from tf_conversions import posemath
from std_msgs.msg import String, Bool, Float32, Empty, Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Quaternion, Wrench, WrenchStamped, TwistStamped
from sensor_msgs.msg import JointState, Joy

class arm(object):
    """Simple arm API wrapping around ROS messages
    """

    # class to contain spatial/body cf methods
    class MeasuredServoCf:
        def __init__(self, ros_namespace, expected_interval):
            self.crtk = crtk.utils(self, ros_namespace)
            self.crtk.add_measured_cf()
            self.crtk.add_servo_cf()

    # local kinematics
    class Local:
        def __init__(self, ros_namespace, expected_interval):
            self.crtk = crtk.utils(self, ros_namespace)
            self.crtk.add_measured_cp()
            self.crtk.add_setpoint_cp()

    # initialize the arm
    def __init__(self, arm_name, ros_namespace = '', expected_interval = 0.01):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_arm(arm_name, ros_namespace, expected_interval)


    def __init_arm(self, arm_name, ros_namespace, expected_interval):
        """Constructor.  This initializes a few data members.It
        requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `PSM1`"""
        # data members, event based
        self.__arm_name = arm_name
        self.__ros_namespace = ros_namespace
        self.__full_ros_namespace = ros_namespace + arm_name

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__full_ros_namespace, expected_interval)

        # add crtk features that we need and are supported by the dVRK
        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_setpoint_js()
        self.__crtk_utils.add_setpoint_cp()
        self.__crtk_utils.add_measured_js()
        self.__crtk_utils.add_measured_cp()
        self.__crtk_utils.add_measured_cv()
        self.__crtk_utils.add_servo_jp()
        self.__crtk_utils.add_servo_jr()
        self.__crtk_utils.add_servo_cp()
        self.__crtk_utils.add_servo_jf()
        self.__crtk_utils.add_move_jp()
        self.__crtk_utils.add_move_jr()
        self.__crtk_utils.add_move_cp()

        self.spatial = self.MeasuredServoCf(self.__full_ros_namespace + '/spatial', expected_interval)
        self.body = self.MeasuredServoCf(self.__full_ros_namespace + '/body', expected_interval)
        self.local = self.Local(self.__full_ros_namespace + '/local', expected_interval)

        # continuous publish from dvrk_bridge
        self.__jacobian_spatial = numpy.ndarray(0, dtype = numpy.float)
        self.__jacobian_body = numpy.ndarray(0, dtype = numpy.float)

        self.__sub_list = []
        self.__pub_list = []

        # publishers
        frame = PyKDL.Frame()
        self.__set_wrench_body_orientation_absolute_pub = rospy.Publisher(self.__full_ros_namespace
                                                                          + '/set_wrench_body_orientation_absolute',
                                                                          Bool, latch = True, queue_size = 1)
        self.__set_gravity_compensation_pub = rospy.Publisher(self.__full_ros_namespace
                                                              + '/set_gravity_compensation',
                                                              Bool, latch = True, queue_size = 1)
        self.__pub_list = [self.__set_wrench_body_orientation_absolute_pub,
                           self.__set_gravity_compensation_pub]
        # subscribers
        self.__sub_list = [rospy.Subscriber(self.__full_ros_namespace + '/jacobian_spatial',
                                            Float64MultiArray, self.__jacobian_spatial_cb),
                           rospy.Subscriber(self.__full_ros_namespace + '/jacobian_body',
                                            Float64MultiArray, self.__jacobian_body_cb)]

        # create node
        if not rospy.get_node_uri():
            rospy.init_node('arm_api', anonymous = True, log_level = rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')


    def __jacobian_spatial_cb(self, data):
        """Callback for the Jacobian in spatial frame.

        :param data: Jacobian."""
        jacobian = numpy.asarray(data.data)
        jacobian.shape = data.layout.dim[0].size, data.layout.dim[1].size
        self.__jacobian_spatial = jacobian


    def __jacobian_body_cb(self, data):
        """Callback for the Jacobian in spatial frame.

        :param data: Jacobian."""
        jacobian = numpy.asarray(data.data)
        jacobian.shape = data.layout.dim[0].size, data.layout.dim[1].size
        self.__jacobian_body = jacobian


    def name(self):
        return self.__arm_name


    def set_wrench_body_orientation_absolute(self, absolute):
        """Apply body wrench using body orientation (relative/False) or reference frame (absolute/True)"""
        m = Bool()
        m.data = absolute
        self.__set_wrench_body_orientation_absolute_pub.publish(m)


    def set_gravity_compensation(self, gravity_compensation):
        "Turn on/off gravity compensation in cartesian effort mode"
        g = Bool()
        g.data = gravity_compensation
        self.__set_gravity_compensation_pub.publish(g)


    def unregister(self, verbose=False):
        for sub in self.__sub_list:
            sub.unregister()
        if verbose:
            print('Unregistered {} subs for {}'.format(self.__sub_list.__len__(), self.__arm_name))

        for pub in self.__pub_list:
            pub.unregister()
        if verbose:
            print('Unregistered {} pubs for {}'.format(self.__pub_list.__len__(), self.__arm_name))
