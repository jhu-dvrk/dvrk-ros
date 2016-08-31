#  Author(s):  Anton Deguet
#  Created on: 2016-05

#   (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

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

.. _interpolate:

Interpolation
=============

If the `interpolation` flag is set to `True` (default), the arm
controller will use a `trajectory generator
<http://ttuadvancedrobotics.wikidot.com/trajectory-planning-for-point-to-point-motion>`_
to create set points between the current position and the position
requested by the user.  If your desired position is "far" from the
current position, you should always set the `interpolate` flag to
`True`.

The only case where you should override the default and set
`interpolate` to `False` is if you are sending positions close to each
other.  For example, when `tele-operating
<https://en.wikipedia.org/wiki/Teleoperation>`_, all the master
positions you will receive will define a continuous trajectory with
positions close to each other.

It is important to note that when `interpolate` is set to `False`,
sending a new goal that is far from the last desired position will
likely trigger a `PID tracking error <https://en.wikipedia.org/wiki/PID_controller>`_.

.. _currentvdesired:

Current vs Desired position
===========================

The arm controller can provide two different positions at any given
time.  The current position is the position measured by the sensors
(in most cases, encoders).  This position defines the physical
position of the system.  The desired joint position is the position
sent to the low level controller (e.g. `PID
<https://en.wikipedia.org/wiki/PID_controller>`_).  The desired
cartesian position is calculted using the desired joint position.
When using a `trajectory
<http://ttuadvancedrobotics.wikidot.com/trajectory-planning-for-point-to-point-motion>`_,
the desired position is not the final goal but the last set point
generated for the trajectory.

Desired positions might differ from the physical positions due to
`forces (gravity, friction, ...) <https://en.wikipedia.org/wiki/Force>`_ applied on the arm.  When
implementing an incremental move, one should always use the last
desired position.  If one needs to track the arm, it is better to
use the current position.

Arm API
=========

"""

# sphinx-apidoc -F -A "Yijun Hu" -o doc src

import inspect
import threading
import math

import rospy
import numpy
import PyKDL

# we should probably not import the symbols and put them in current namespace
from tf import transformations
from tf_conversions import posemath
from std_msgs.msg import String, Bool, Float32, Empty
from geometry_msgs.msg import Pose, PoseStamped, Vector3, Quaternion, Wrench, WrenchStamped, TwistStamped
from sensor_msgs.msg import JointState, Joy

# from code import InteractiveConsole
# from imp import new_module

#class Console(InteractiveConsole):
#    def __init__(self, names=None):
#        names = names or {}
#        names['console'] = self
#        InteractiveConsole.__init__(self, names)
#        self.superspace = new_module('superspace')
#
#    def enter(self, source):
#        source = self.preprocess(source)
#        self.runcode(source)
#
#    @staticmethod
#    def preprocess(source):
#        return source

class arm(object):
    """Simple arm API wrapping around ROS messages
    """

    # initialize the arm
    def __init__(self, arm_name, ros_namespace = '/dvrk/'):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_arm(arm_name, ros_namespace)


    def __init_arm(self, arm_name, ros_namespace = '/dvrk/'):
        """Constructor.  This initializes a few data members.It
        requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `/dvrk/PSM1`"""
        # data members, event based
        self.__arm_name = arm_name
        self.__ros_namespace = ros_namespace
        self.__robot_state = 'uninitialized'
        self.__robot_state_event = threading.Event()
        self.__goal_reached = False
        self.__goal_reached_event = threading.Event()

        # continuous publish from dvrk_bridge
        self.__position_joint_desired = numpy.array(0, dtype = numpy.float)
        self.__effort_joint_desired = numpy.array(0, dtype = numpy.float)
        self.__position_cartesian_desired = PyKDL.Frame()
        self.__position_cartesian_local_desired = PyKDL.Frame()
        self.__position_joint_current = numpy.array(0, dtype = numpy.float)
        self.__velocity_joint_current = numpy.array(0, dtype = numpy.float)
        self.__effort_joint_current = numpy.array(0, dtype = numpy.float)
        self.__position_cartesian_current = PyKDL.Frame()
        self.__position_cartesian_local_current = PyKDL.Frame()
        self.__twist_body_current = numpy.zeros(6, dtype = numpy.float)
        self.__wrench_body_current = numpy.zeros(6, dtype = numpy.float)

        # publishers
        frame = PyKDL.Frame()
        self.__full_ros_namespace = self.__ros_namespace + self.__arm_name
        self.__set_robot_state_pub = rospy.Publisher(self.__full_ros_namespace
                                                     + '/set_robot_state',
                                                     String, latch = True, queue_size = 1)
        self.__set_position_joint_pub = rospy.Publisher(self.__full_ros_namespace
                                                        + '/set_position_joint',
                                                        JointState, latch = True, queue_size = 1)
        self.__set_position_goal_joint_pub = rospy.Publisher(self.__full_ros_namespace
                                                             + '/set_position_goal_joint',
                                                             JointState, latch = True, queue_size = 1)
        self.__set_position_cartesian_pub = rospy.Publisher(self.__full_ros_namespace
                                                            + '/set_position_cartesian',
                                                            Pose, latch = True, queue_size = 1)
        self.__set_position_goal_cartesian_pub = rospy.Publisher(self.__full_ros_namespace
                                                                 + '/set_position_goal_cartesian',
                                                                 Pose, latch = True, queue_size = 1)
        self.__set_wrench_body_pub = rospy.Publisher(self.__full_ros_namespace
                                                     + '/set_wrench_body',
                                                     Wrench, latch = True, queue_size = 1)
        self.__set_wrench_body_orientation_absolute_pub = rospy.Publisher(self.__full_ros_namespace
                                                                          + '/set_wrench_body_orientation_absolute',
                                                                          Bool, latch = True, queue_size = 1)
        self.__set_wrench_spatial_pub = rospy.Publisher(self.__full_ros_namespace
                                                        + '/set_wrench_spatial',
                                                        Wrench, latch = True, queue_size = 1)
        self.__set_gravity_compensation_pub = rospy.Publisher(self.__full_ros_namespace
                                                              + '/set_gravity_compensation',
                                                              Bool, latch = True, queue_size = 1)
        # subscribers
        rospy.Subscriber(self.__full_ros_namespace + '/robot_state',
                         String, self.__robot_state_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/goal_reached',
                         Bool, self.__goal_reached_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/state_joint_desired',
                         JointState, self.__state_joint_desired_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/position_cartesian_desired',
                         PoseStamped, self.__position_cartesian_desired_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/position_cartesian_local_desired',
                         PoseStamped, self.__position_cartesian_local_desired_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/state_joint_current',
                         JointState, self.__state_joint_current_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/position_cartesian_current',
                         PoseStamped, self.__position_cartesian_current_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/position_cartesian_local_current',
                         PoseStamped, self.__position_cartesian_local_current_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/twist_body_current',
                         TwistStamped, self.__twist_body_current_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/wrench_body_current',
                         WrenchStamped, self.__wrench_body_current_cb)

        # create node
        rospy.init_node('arm_api', anonymous = True, log_level = rospy.WARN)


    def __robot_state_cb(self, data):
        """Callback for arm state.

        :param data: the current arm state"""
        self.__robot_state = data.data
        self.__robot_state_event.set()


    def __goal_reached_cb(self, data):
        """Callback for the goal reached.

        :param data: the goal reached"""
        self.__goal_reached = data.data
        self.__goal_reached_event.set()


    def __state_joint_desired_cb(self, data):
        """Callback for the joint desired position.

        :param data: the `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_desired"""
        self.__position_joint_desired.resize(len(data.position))
        self.__effort_joint_desired.resize(len(data.effort))
        self.__position_joint_desired.flat[:] = data.position
        self.__effort_joint_desired.flat[:] = data.effort


    def __position_cartesian_desired_cb(self, data):
        """Callback for the cartesian desired position.

        :param data: the cartesian position desired"""
        self.__position_cartesian_desired = posemath.fromMsg(data.pose)


    def __position_cartesian_local_desired_cb(self, data):
        """Callback for the cartesian desired position.

        :param data: the cartesian position desired"""
        self.__position_cartesian_local_desired = posemath.fromMsg(data.pose)


    def __state_joint_current_cb(self, data):
        """Callback for the current joint position.

        :param data: the `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_current"""
        self.__position_joint_current.resize(len(data.position))
        self.__velocity_joint_current.resize(len(data.velocity))
        self.__effort_joint_current.resize(len(data.effort))
        self.__position_joint_current.flat[:] = data.position
        self.__velocity_joint_current.flat[:] = data.velocity
        self.__effort_joint_current.flat[:] = data.effort


    def __position_cartesian_current_cb(self, data):
        """Callback for the current cartesian position.

        :param data: The cartesian position current."""
        self.__position_cartesian_current = posemath.fromMsg(data.pose)


    def __position_cartesian_local_current_cb(self, data):
        """Callback for the current cartesian position.

        :param data: The cartesian position current."""
        self.__position_cartesian_local_current = posemath.fromMsg(data.pose)


    def __twist_body_current_cb(self, data):
        """Callback for the current twist in body frame.

        :param data: Twist."""
        self.__twist_body_current[0] = data.twist.linear.x
        self.__twist_body_current[1] = data.twist.linear.y
        self.__twist_body_current[2] = data.twist.linear.z
        self.__twist_body_current[3] = data.twist.angular.x
        self.__twist_body_current[4] = data.twist.angular.y
        self.__twist_body_current[5] = data.twist.angular.z


    def __wrench_body_current_cb(self, data):
        """Callback for the current wrench in body frame.

        :param data: Wrench."""
        self.__wrench_body_current[0] = data.wrench.force.x
        self.__wrench_body_current[1] = data.wrench.force.y
        self.__wrench_body_current[2] = data.wrench.force.z
        self.__wrench_body_current[3] = data.wrench.torque.x
        self.__wrench_body_current[4] = data.wrench.torque.y
        self.__wrench_body_current[5] = data.wrench.torque.z


    def __dvrk_set_state(self, state, timeout = 5):
        """Set state with block.

        :param state: the arm state
        :param timeout: the lenghth you want to wait for arm to change state
        :return: whether or not the arm state has been successfuly set
        :rtype: Bool"""
        if (self.__robot_state == state):
            return True
        self.__robot_state_event.clear()
        self.__set_robot_state_pub.publish(state)
        self.__robot_state_event.wait(timeout)
        # if the state is not changed return False
        if (self.__robot_state != state):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state ' + state)
            return False
        return True


    def home(self):
        """This method will provide power to the arm and will home
        the arm."""
        self.__robot_state_event.clear()
        self.__set_robot_state_pub.publish('Home')
        counter = 10 # up to 10 transitions to get ready
        while (counter > 0):
            self.__robot_state_event.wait(20) # give up to 20 secs for each transition
            if (self.__robot_state != 'DVRK_READY'):
                self.__robot_state_event.clear()
                counter = counter - 1
            else:
                counter = -1
        if (self.__robot_state != 'DVRK_READY'):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state DVRK_READY')


    def shutdown(self):
        """Stop providing power to the arm."""
        self.__dvrk_set_state('DVRK_UNINITIALIZED', 20)


    def get_robot_state(self):
        """Get the robot state.
        :returns: the robot state
        :rtype: string"""
        return self.__robot_state


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


    def get_current_twist_body(self):
        """Get the current cartesian velocity of the arm.  This
        is based on the body jacobian, both linear and angular are
        rotated to be defined in base frame.

        :returns: the current position of the arm in cartesian space
        :rtype: geometry_msgs.TwistStamped"""
        return self.__twist_body_current


    def get_current_wrench_body(self):
        """Get the current cartesian force applied on arm.  This is
        based on the body jacobian, both linear and angular are
        rotated to be defined in base frame if the flag
        wrench_body_orientation_absolute is set to True.  See method
        set_wrench_body_orientation_absolute.

        :returns: the current force applied to the arm in cartesian space
        :rtype: geometry_msgs.WrenchStamped"""
        return self.__wrench_body_current


    def get_current_joint_position(self):
        """Get the :ref:`current joint position <currentvdesired>` of
        the arm.

        :returns: the current position of the arm in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__position_joint_current


    def get_current_joint_velocity(self):
        """Get the :ref:`current joint velocity <currentvdesired>` of
        the arm.

        :returns: the current position of the arm in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__velocity_joint_current


    def get_current_joint_effort(self):
        """Get the :ref:`current joint effort <currentvdesired>` of
        the arm.

        :returns: the current position of the arm in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__effort_joint_current


    def get_desired_position(self):
        """Get the :ref:`desired cartesian position <currentvdesired>` of the arm.

        :returns: the desired position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__position_cartesian_desired


    def get_desired_position_local(self):
        """Get the :ref:`desired cartesian position <currentvdesired>` of the arm.

        :returns: the desired position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__position_cartesian_local_desired


    def get_desired_joint_position(self):
        """Get the :ref:`desired joint position <currentvdesired>` of
        the arm.

        :returns: the desired position of the arm in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__position_joint_desired


    def get_desired_joint_effort(self):
        """Get the :ref:`desired joint effort <currentvdesired>` of
        the arm.

        :returns: the desired effort of the arm in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__effort_joint_desired


    def get_joint_number(self):
        """Get the number of joints on the arm specified.

        :returns: the number of joints on the specified arm
        :rtype: int"""
        joint_num = len(self.__position_joint_desired)
        return joint_num


    def __check_input_type(self, input, type_list):
        """Check if the data input is a data type that is located in type_list

        :param input: The data type that needs to be checked.
        :param type_list : A list of types to check input against.
        :returns: whether or not the input is a type in type_list
        :rtype: Bool"""
        found = False
        # check the input against all input_type
        for i in range (len(type_list)):
            if (type(input) is type_list[i]):
                  return True
        # not of type_list print state for this error inside
        if (found == False):
            print 'Error in ', inspect.stack()[1][3], 'input is of type', input, 'and is not one of:'
            message = ''
            # skip_length
            i = 0
            while i < len(type_list):
                message += ' '
                message += str(type_list[i])
                i += 1
            print message
        return False


    def dmove(self, delta_input, interpolate = True):
        """Incremental motion in cartesian space.

        :param delta_input: the incremental motion you want to make
        :param interpolate: see  :ref:`interpolate <interpolate>`
        """
        # is this a legal translation input
        if (self.__check_input_type(delta_input, [PyKDL.Vector, PyKDL.Rotation, PyKDL.Frame])):
            if (type(delta_input) is PyKDL.Vector):
                return self.__dmove_translation(delta_input, interpolate)
            elif (type(delta_input) is PyKDL.Rotation):
                return self.__dmove_rotation(delta_input, interpolate)
            elif (type(delta_input) is PyKDL.Frame):
                return self.__dmove_frame(delta_input, interpolate)


    def __dmove_translation(self, delta_translation, interpolate = True):
        """Incremental translation (using PYKDL Vector) in cartesian space.

        :param delta_translation: the incremental translation you want to make based on the current position, this is in terms of a  `PyKDL.Vector <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        # convert into a Frame
        delta_rotation = PyKDL.Rotation.Identity()
        delta_frame = PyKDL.Frame(delta_rotation, delta_translation)
        return self.__dmove_frame(delta_frame, interpolate)


    def __dmove_rotation(self, delta_rotation, interpolate = True):
        """Incremental rotation (using PyKDL Rotation) in cartesian plane.

        :param delta_rotation: the incremental `PyKDL.Rotation <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_ based upon the current position
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        # convert into a Frame
        delta_vector = PyKDL.Vector(0.0, 0.0, 0.0)
        delta_frame = PyKDL.Frame(delta_rotation, delta_vector)
        return self.__dmove_frame(delta_frame, interpolate)


    def __dmove_frame(self, delta_frame, interpolate = True):
        """Incremental move (using PyKDL Frame) in cartesian plane.

        :param delta_frame: the incremental `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_ based upon the current position
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        # add the incremental move to the current position, to get the ending frame
        end_frame = delta_frame * self.__position_cartesian_desired
        return self.__move_frame(end_frame, interpolate)


    def move(self, abs_input, interpolate = True):
        """Absolute translation in cartesian space.

        :param abs_input: the absolute translation you want to make
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        # is this a legal translation input
        if (self.__check_input_type(abs_input, [PyKDL.Vector, PyKDL.Rotation, PyKDL.Frame])):
            if (type(abs_input) is PyKDL.Vector):
                return self.__move_translation(abs_input, interpolate)
            elif (type(abs_input) is PyKDL.Rotation):
                return self.__move_rotation(abs_input, interpolate)
            elif (type(abs_input) is PyKDL.Frame):
                return self.__move_frame(abs_input, interpolate)


    def __move_translation(self, abs_translation, interpolate = True):
        """Absolute translation in cartesian space.

        :param abs_translation: the absolute translation you want to make based on the current position, this is in terms of a  `PyKDL.Vector <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        # convert into a Frame
        abs_rotation = self.__position_cartesian_desired.M
        abs_frame = PyKDL.Frame(abs_rotation, abs_translation)
        return self.__move_frame(abs_frame, interpolate)


    def __move_rotation(self, abs_rotation, interpolate = True):
        """Absolute rotation in cartesian space.

        :param abs_rotation: the absolute `PyKDL.Rotation <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        # convert into a Frame
        abs_vector = self.__position_cartesian_desired.p
        abs_frame = PyKDL.Frame(abs_rotation, abs_vector)
        return self.__move_frame(abs_frame, interpolate)


    def __move_frame(self, abs_frame, interpolate = True):
        """Absolute move by PyKDL.Frame in Cartesian space.

        :param abs_frame: the absolute `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        # move based on value of interpolate
        if (interpolate):
            return self.__move_cartesian_goal(abs_frame)
        else:
            return self.__move_cartesian_direct(abs_frame)


    def __move_cartesian_direct(self, end_frame):
        """Move the arm to the end position by passing the trajectory generator.

        :param end_frame: the ending `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :returns: true if you had successfully move
        :rtype: Bool"""
        # set in position cartesian mode
        end_position = posemath.toMsg(end_frame)
        if (not self.__dvrk_set_state('DVRK_POSITION_CARTESIAN')):
            return False
        # go to that position directly
        self.__set_position_cartesian_pub.publish(end_position)
        return True


    def __move_cartesian_goal(self, end_frame):
        """Move the arm to the end position by providing a goal for trajectory generator.

        :param end_frame: the ending `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :returns: true if you had succesfully move
        :rtype: Bool"""
        # set in position cartesian mode
        end_position= posemath.toMsg(end_frame)
        if (not self.__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        # go to that position by goal
        return self.__set_position_goal_cartesian_publish_and_wait(end_position)


    def __set_position_goal_cartesian_publish_and_wait(self, end_position):
        """Wrapper around publisher/subscriber to manage events for cartesian coordinates.

        :param end_position: the ending `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :returns: returns true if the goal is reached
        :rtype: Bool"""
        self.__goal_reached_event.clear()
        # the goal is originally not reached
        self.__goal_reached = False
        # recursively call this function until end is reached
        self.__set_position_goal_cartesian_pub.publish(end_position)
        self.__goal_reached_event.wait(20) # 1 minute at most
        if not self.__goal_reached:
            return False
        return True


    def dmove_joint(self, delta_pos, interpolate = True):
        """Incremental move in joint space.

        :param delta_pos: the incremental amount in which you want to move index by, this is in terms of a numpy array
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        if ((not(type(delta_pos) is numpy.ndarray))
             or (not(delta_pos.dtype == numpy.float64))):
            print "delta_pos must be an array of floats"
            return False
        if (not(delta_pos.size ==  self.get_joint_number())):
            print "delta_pos must be an array of size", self.get_joint_number()
            return False

        abs_pos = numpy.array(self.__position_joint_desired)
        abs_pos = abs_pos+ delta_pos
        return self.__move_joint(abs_pos, interpolate)


    def dmove_joint_one(self, delta_pos, indices, interpolate = True):
        """Incremental index move of 1 joint in joint space.

        :param delta_pos: the incremental amount in which you want to move index by, this is a float
        :param index: the joint you want to move, this is an integer
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        if (type(delta_pos) is float and type(indices) is int):
            return self.dmove_joint_some(numpy.array([delta_pos]), numpy.array([indices]), interpolate)
        else:
            return False


    def dmove_joint_some(self, delta_pos, indices, interpolate = True):
        """Incremental index move of a series of joints in joint space.

        :param delta_pos: the incremental amount in which you want to move index by, this is a numpy array corresponding to the number of indices
        :param indices: the joints you want to move, this is a numpy array of indices
        :param interpolate: see  :ref:`interpolate <interpolate>`"""

        # check if delta is an array
        if ((not(type(delta_pos) is numpy.ndarray))
             or (not(delta_pos.dtype == numpy.float64))):
            print "delta_pos must be an array of floats"
            return False

        # check the length of the delta move
        if ((not(type(indices) is numpy.ndarray))
            or (not(indices.dtype == numpy.int64))):
            print "indices must be an array of integers"
            return False

        if ((not(indices.size == delta_pos.size))
            or (indices.size > self.get_joint_number())):
            print "size of delta_pos and indices must match and be less than", self.get_joint_number()
            return False

        for i in range(indices.size):
            if (indices[i] > self.get_joint_number()):
                print "all indices must be less than", self.get_joint_number()
                return False

        abs_pos = numpy.array(self.__position_joint_desired)
        for i in range(len(indices)):
            abs_pos[indices[i]] = abs_pos[indices[i]] + delta_pos[i]

        # move accordingly
        return self.__move_joint(abs_pos, interpolate)


    def move_joint(self, abs_pos, interpolate = True):
        """Absolute move in joint space.

        :param abs_pos: the absolute position in which you want to move, this is a numpy array
        :param interpolate: see  :ref:`interpolate <interpolate>`"""

        if ((not(type(abs_pos) is numpy.ndarray))
            or (not(abs_pos.dtype == numpy.float64))):
            print "abs_pos must be an array of floats"
            return False
        if (not(abs_pos.size == self.get_joint_number())):
            print "abs_pos must be an array of size", self.get_joint_number()
            return False

        return self.__move_joint(abs_pos, interpolate)


    def move_joint_one(self, abs_pos, joint_index, interpolate = True):
        """Absolute index move of 1 joint in joint space.

        :param value: the absolute amount in which you want to move index by, this is a list
        :param index: the joint you want to move, this is a list
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        if ((type(abs_pos) is float) and (type(joint_index) is int)):
            return self.move_joint_some(numpy.array([abs_pos]), numpy.array([joint_index]), interpolate)
        else:
            return False


    def move_joint_some(self, abs_pos, indices, interpolate = True):
        """Absolute index move of a series of joints in joint space.

        :param value: the absolute amount in which you want to move index by, this is a list
        :param index: the joint you want to move, this is a list
        :param interpolate: see  :ref:`interpolate <interpolate>`"""

        if ((not(type(abs_pos) is numpy.ndarray))
            or (not(abs_pos.dtype == numpy.float64))):
            print "delta_pos must be an array of floats"
            return False

        # check the length of the delta move
        if ((not(type(indices) is numpy.ndarray))
            or (not(indices.dtype == numpy.int64))):
            print "indices must be an array of integers"
            return False

        if ((not(indices.size == abs_pos.size))
            or (indices.size > self.get_joint_number())):
            print "size of delta_pos and indices must match and be less than", self.get_joint_number()
            return False

        for i in range(indices.size):
            if (indices[i] > self.get_joint_number()):
                print "all indices must be less than", self.get_joint_number()
                return False

        abs_pos_result = numpy.array(self.__position_joint_desired)
        for i in range(len(indices)):
            abs_pos_result[indices[i]] = abs_pos[i]

        # move accordingly
        return self.__move_joint(abs_pos_result, interpolate)


    def __move_joint(self, abs_joint, interpolate = True):
        """Absolute move by vector in joint plane.

        :param abs_joint: the absolute position of the joints in terms of a numpy array
        :param interpolate: if false the trajectory generator will be used; if true you can bypass the trajectory generator"""
        if (interpolate):
            return self.__move_joint_goal(abs_joint)
        else:
            return self.__move_joint_direct(abs_joint)

    def __move_joint_direct(self, end_joint):
        """Move the arm to the end vector by passing the trajectory generator.

        :param end_joint: the list of joints in which you should conclude movement
        :returns: true if you had succesfully move
        :rtype: Bool"""
        if not self.__dvrk_set_state('DVRK_POSITION_JOINT'):
            return False
        # go to that position directly
        joint_state = JointState()
        joint_state.position[:] = end_joint.flat
        self.__set_position_joint_pub.publish(joint_state)
        return True


    def __move_joint_goal(self, end_joint):
        """Move the arm to the end vector by bypassing the trajectory generator.

        :param end_joint: the list of joints in which you should conclude movement
        :returns: true if you had succesfully move
        :rtype: Bool"""
        if (not self.__dvrk_set_state('DVRK_POSITION_GOAL_JOINT')):
            return False
        joint_state = JointState()
        joint_state.position[:] = end_joint.flat
        return self.__set_position_goal_joint_publish_and_wait(joint_state)


    def __set_position_goal_joint_publish_and_wait(self, end_position):
        """Wrapper around publisher/subscriber to manage events for joint coordinates.

        :param end_position: there is only one parameter, end_position which tells us what the ending position is
        :returns: whether or not you have successfully moved by goal or not
        :rtype: Bool"""
        self.__goal_reached_event.clear()
        self.__goal_reached = False
        self.__set_position_goal_joint_pub.publish(end_position)
        self.__goal_reached_event.wait(20) # 1 minute at most
        if not self.__goal_reached:
            return False
        return True


    def set_wrench_spatial_force(self, force):
        """Apply a wrench with force only (spatial), torque is null

        :param force: the new force to set it to
        """
        if (not self.__dvrk_set_state('DVRK_EFFORT_CARTESIAN')):
            return False
        w = Wrench()
        w.force.x = force[0]
        w.force.y = force[1]
        w.force.z = force[2]
        w.torque.x = 0.0
        w.torque.y = 0.0
        w.torque.z = 0.0
        self.__set_wrench_spatial_pub.publish(w)


    def set_wrench_body_orientation_absolute(self, absolute):
        """Apply body wrench using body orientation (relative/False) or reference frame (absolute/True)"""
        m = Bool()
        m.data = absolute
        self.__set_wrench_body_orientation_absolute_pub.publish(m)


    def set_wrench_body_force(self, force):
        "Apply a wrench with force only (body), torque is null"
        if (not self.__dvrk_set_state('DVRK_EFFORT_CARTESIAN')):
            return False
        w = Wrench()
        w.force.x = force[0]
        w.force.y = force[1]
        w.force.z = force[2]
        w.torque.x = 0.0
        w.torque.y = 0.0
        w.torque.z = 0.0
        self.__set_wrench_body_pub.publish(w)


    def set_gravity_compensation(self, gravity_compensation):
        "Turn on/off gravity compensation in cartesian effort mode"
        g = Bool()
        g.data = gravity_compensation
        self.__set_gravity_compensation_pub.publish(g)
