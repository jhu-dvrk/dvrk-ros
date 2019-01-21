#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2019 Johns Hopkins University (JHU), All Rights Reserved.

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

    # initialize the arm
    def __init__(self, arm_name, ros_namespace = 'dvrk/'):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_arm(arm_name, ros_namespace)


    def __init_arm(self, arm_name, ros_namespace = 'dvrk/'):
        """Constructor.  This initializes a few data members. It requires an
        arm name, this will be used to find the ROS topics for the arm
        being controlled.  For example if the arm name is `PSM1`, the
        ROS topics will be from the namespace `dvrk/PSM1`.

        """

        # data members, event based
        self.__arm_name = arm_name
        self.__ros_namespace = ros_namespace

        # crtk features
        self.__crtk_utils = crtk.utils(self, ros_namespace + arm_name)

        # add crtk features that we need and are supported by the dVRK
        self.__crtk_utils.add_setpoint_js(self)
        self.__crtk_utils.add_setpoint_cp(self)
        self.__crtk_utils.add_measured_js(self)
        self.__crtk_utils.add_measured_cp(self)
        self.__crtk_utils.add_measured_cv(self)
        self.__crtk_utils.add_measured_cf(self)
        self.__crtk_utils.add_servo_jp(self)
        self.__crtk_utils.add_servo_cp(self)
        self.__crtk_utils.add_servo_jf(self)
        self.__crtk_utils.add_servo_cf(self)
        self.__crtk_utils.add_move_jp(self)
        self.__crtk_utils.add_move_cp(self)

        # non crtk topics
        self.__arm_current_state = ''
        self.__arm_current_state_event = threading.Event()
        self.__arm_desired_state = ''
        self.__goal_reached = False
        self.__goal_reached_event = threading.Event()

        # continuous publish from dvrk_bridge
        self.__jacobian_spatial = numpy.ndarray(0, dtype = numpy.float)
        self.__jacobian_body = numpy.ndarray(0, dtype = numpy.float)

        self.__sub_list = []
        self.__pub_list = []

        # publishers
        frame = PyKDL.Frame()
        self.__full_ros_namespace = self.__ros_namespace + self.__arm_name
        self.__set_arm_desired_state_pub = rospy.Publisher(self.__full_ros_namespace
                                                           + '/set_desired_state',
                                                           String, latch = True, queue_size = 1)
        self.__set_wrench_body_orientation_absolute_pub = rospy.Publisher(self.__full_ros_namespace
                                                                          + '/set_wrench_body_orientation_absolute',
                                                                          Bool, latch = True, queue_size = 1)
        self.__set_gravity_compensation_pub = rospy.Publisher(self.__full_ros_namespace
                                                              + '/set_gravity_compensation',
                                                              Bool, latch = True, queue_size = 1)
        self.__pub_list = [self.__set_arm_desired_state_pub,
                           self.__set_wrench_body_orientation_absolute_pub,
                           self.__set_gravity_compensation_pub]
        # subscribers
        self.__sub_list = [rospy.Subscriber(self.__full_ros_namespace + '/current_state',
                                            String, self.__arm_current_state_cb),
                           rospy.Subscriber(self.__full_ros_namespace + '/desired_state',
                                          String, self.__arm_desired_state_cb),
                           rospy.Subscriber(self.__full_ros_namespace + '/goal_reached',
                                          Bool, self.__goal_reached_cb),
                           rospy.Subscriber(self.__full_ros_namespace + '/jacobian_spatial',
                                          Float64MultiArray, self.__jacobian_spatial_cb),
                           rospy.Subscriber(self.__full_ros_namespace + '/jacobian_body',
                                          Float64MultiArray, self.__jacobian_body_cb)]

        # create node
        if not rospy.get_node_uri():
            rospy.init_node('arm_api', anonymous = True, log_level = rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')


    def __arm_current_state_cb(self, data):
        """Callback for arm current state.

        :param data: the current arm state"""
        self.__arm_current_state = data.data
        self.__arm_current_state_event.set()


    def __arm_desired_state_cb(self, data):
        """Callback for arm desired state.

        :param data: the desired arm state"""
        self.__arm_desired_state = data.data


    def __goal_reached_cb(self, data):
        """Callback for the goal reached.

        :param data: the goal reached"""
        self.__goal_reached = data.data
        self.__goal_reached_event.set()


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

    def __set_desired_state(self, state, timeout = 5):
        """Set state with block.

        :param state: the desired arm state
        :param timeout: the amount of time you want to wait for arm to change state
        :return: whether or not the arm state has been successfuly set
        :rtype: Bool"""
        if (self.__arm_desired_state == state):
            return True
        self.__arm_current_state_event.clear()
        self.__set_arm_desired_state_pub.publish(state)
        self.__arm_current_state_event.wait(timeout)
        # if the state is not changed return False
        if (self.__arm_current_state != state):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state ' + state)
            return False
        return True


    def name(self):
        return self.__arm_name


    def home(self):
        """This method will provide power to the arm and will home
        the arm."""
        # if we already received a state
        if (self.__arm_current_state == 'READY'):
            return
        self.__arm_current_state_event.clear()
        self.__set_arm_desired_state_pub.publish('READY')
        counter = 10 # up to 10 transitions to get ready
        while (counter > 0):
            self.__arm_current_state_event.wait(20) # give up to 20 secs for each transition
            if (self.__arm_current_state != 'READY'):
                self.__arm_current_state_event.clear()
                counter = counter - 1
            else:
                counter = -1
        if (self.__arm_current_state != 'READY'):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state READY')


    def shutdown(self):
        """Stop providing power to the arm."""
        self.__set_desired_state('UNINITIALIZED', 20)


    def get_arm_current_state(self):
        """Get the arm current state.
        :returns: the arm current state
        :rtype: string"""
        return self.__arm_current_state


    def get_arm_desired_state(self):
        """Get the arm desired state.
        :returns: the arm desired state
        :rtype: string"""
        return self.__arm_desired_state


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
