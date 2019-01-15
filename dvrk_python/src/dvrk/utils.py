#  Author(s):  Anton Deguet
#  Created on: 2018-02-15

# (C) Copyright 2018 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import threading
import time

import rospy
import numpy
import PyKDL
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg


def TransformFromMsg(t):
    """
    :param p: input pose
    :type p: :class:`geometry_msgs.msg.Pose`
    :return: New :class:`PyKDL.Frame` object

    Convert a pose represented as a ROS Pose message to a :class:`PyKDL.Frame`.
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.rotation.x,
                                                 t.rotation.y,
                                                 t.rotation.z,
                                                 t.rotation.w),
                       PyKDL.Vector(t.translation.x,
                                    t.translation.y,
                                    t.translation.z))

def TransformToMsg(f):
    """
    :param f: input pose
    :type f: :class:`PyKDL.Frame`

    Return a ROS Pose message for the Frame f.

    """
    m = geometry_msgs.msg.TransformStamped()
    t = m.transform
    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = f.M.GetQuaternion()
    t.translation.x = f.p[0]
    t.translation.y = f.p[1]
    t.translation.z = f.p[2]
    return m



class utils:
    def __init__(self, ros_namespace):
        self.__ros_namespace = ros_namespace
        self.__subscribers = []
        self.__publishers = []
        # internal data for subscriber callbacks
        self.__setpoint_jp_data = numpy.array(0, dtype = numpy.float)
        self.__setpoint_jf_data = numpy.array(0, dtype = numpy.float)
        self.__setpoint_cp_data = PyKDL.Frame()
        self.__measured_jp_data = numpy.array(0, dtype = numpy.float)
        self.__measured_jv_data = numpy.array(0, dtype = numpy.float)
        self.__measured_jf_data = numpy.array(0, dtype = numpy.float)
        self.__measured_cp_data = PyKDL.Frame()
        self.__measured_cv_data = numpy.zeros(6, dtype = numpy.float)
        self.__measured_cf_data = numpy.zeros(6, dtype = numpy.float)
        # thread event for blocking commands
        self.__device_state_event = threading.Event()
        self.__is_moving_event = threading.Event()


    # internal methods to manage state
    def __device_state_cb(self, msg):
        self.__device_state_data = msg.data
        self.__device_state_event.set()

    def __device_state(self):
        return self.__device_state_data

    def __device_state_wait(self, state, timeout):
        self.__device_state_event.wait(timeout)
        if self.__device_state_data == state:
            return True
        return False

    def __set_device_state(self, state, timeout = 0):
        # clear timeout
        self.__device_state_event.clear()
        # convert to ROS msg and publish
        msg = std_msgs.msg.String()
        msg.data = state
        # publish and wait
        self.__set_device_state_publisher.publish(msg)
        if timeout == 0:
            return True
        return self.__device_state_wait(state, timeout)

    def __enable(self, timeout = 0):
        return self.__set_device_state('ENABLED', timeout)

    def __disable(self, timeout = 0):
        return self.__set_device_state('DISABLED', timeout)

    def add_device_state(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'device_state'):
            raise RuntimeWarning('device_state already exists')
        # create the subscriber/publisher and keep in list
        self.__device_state_data = ''
        self.__device_state_subscriber = rospy.Subscriber(self.__ros_namespace + '/device_state',
                                                          std_msgs.msg.String, self.__device_state_cb)
        self.__subscribers.append(self.__device_state_subscriber)
        self.__set_device_state_publisher = rospy.Publisher(self.__ros_namespace + '/set_device_state',
                                                            std_msgs.msg.String,
                                                            latch = True, queue_size = 1)
        self.__publishers.append(self.__set_device_state_publisher)
        # add attributes to class instance
        class_instance.device_state = self.__device_state
        class_instance.set_device_state = self.__set_device_state
        class_instance.device_state_wait = self.__device_state_wait
        class_instance.enable = self.__enable
        class_instance.disable = self.__disable


    # internal methods to detect moving status
    def __is_moving_cb(self, msg):
        self.__is_moving_data = msg.data
        self.__is_moving_event.set()

    def __is_moving(self):
        return self.__is_moving_data

    def __is_moving_wait(self, timeout):
        start_time = time.time()
        while True:
            self.__is_moving_event.clear()
            self.__is_moving_event.wait(timeout)
            # if not moving we're good
            if not self.__is_moving_data:
                break
            # otherwise, keep waiting a bit more
            timeout = timeout - (time.time() - start_time)
            if timeout <= 0:
                break
        return not self.__is_moving_data

    def add_is_moving(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'is_moving'):
            raise RuntimeWarning('is_moving already exists')
        # create the subscriber/publisher and keep in list
        self.__is_moving_data = False
        self.__is_moving_subscriber = rospy.Subscriber(self.__ros_namespace + '/is_moving',
                                                          std_msgs.msg.Bool, self.__is_moving_cb)
        self.__subscribers.append(self.__is_moving_subscriber)
        # add attributes to class instance
        class_instance.is_moving = self.__is_moving
        class_instance.is_moving_wait = self.__is_moving_wait


    # internal methods for setpoint_js
    def __setpoint_js_cb(self, msg):
        self.__setpoint_jp_data.resize(len(msg.position))
        self.__setpoint_jf_data.resize(len(msg.effort))
        self.__setpoint_jp_data.flat[:] = msg.position
        self.__setpoint_jf_data.flat[:] = msg.effort

    def __setpoint_jp(self):
        return self.__setpoint_jp_data

    def __setpoint_jf(self):
        return self.__setpoint_jf_data

    def add_setpoint_js(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'setpoint_jp'):
            raise RuntimeWarning('setpoint_js already exists')
        # create the subscriber and keep in list
        self.__setpoint_js_subscriber = rospy.Subscriber(self.__ros_namespace + '/setpoint_js',
                                                         sensor_msgs.msg.JointState,
                                                         self.__setpoint_js_cb)
        self.__subscribers.append(self.__setpoint_js_subscriber)
        # add attributes to class instance
        class_instance.setpoint_jp = self.__setpoint_jp
        class_instance.setpoint_jf = self.__setpoint_jf


    # internal methods for setpoint_cp
    def __setpoint_cp_cb(self, msg):
        self.__setpoint_cp_data = TransformFromMsg(msg.transform)

    def __setpoint_cp(self):
        return self.__setpoint_cp_data

    def add_setpoint_cp(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'setpoint_cp'):
            raise RuntimeWarning('setpoint_cp already exists')
        # create the subscriber and keep in list
        self.__setpoint_cp_subscriber = rospy.Subscriber(self.__ros_namespace + '/setpoint_cp',
                                                         geometry_msgs.msg.TransformStamped,
                                                         self.__setpoint_cp_cb)
        self.__subscribers.append(self.__setpoint_cp_subscriber)
        # add attributes to class instance
        class_instance.setpoint_cp = self.__setpoint_cp


    # internal methods for measured_js
    def __measured_js_cb(self, msg):
        self.__measured_jp_data.resize(len(msg.position))
        self.__measured_jv_data.resize(len(msg.position))
        self.__measured_jf_data.resize(len(msg.effort))
        self.__measured_jp_data.flat[:] = msg.position
        self.__measured_jv_data.flat[:] = msg.velocity
        self.__measured_jf_data.flat[:] = msg.effort

    def __measured_jp(self):
        return self.__measured_jp_data

    def __measured_jv(self):
        return self.__measured_jv_data

    def __measured_jf(self):
        return self.__measured_jf_data

    def add_measured_js(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'measured_jp'):
            raise RuntimeWarning('measured_js already exists')
        # create the subscriber and keep in list
        self.__measured_js_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_js',
                                                         sensor_msgs.msg.JointState,
                                                         self.__measured_js_cb)
        self.__subscribers.append(self.__measured_js_subscriber)

        # add attributes to class instance
        class_instance.measured_jp = self.__measured_jp
        class_instance.measured_jv = self.__measured_jv
        class_instance.measured_jf = self.__measured_jf


    # internal methods for measured_cp
    def __measured_cp_cb(self, msg):
        self.__measured_cp_data = TransformFromMsg(msg.transform)

    def __measured_cp(self):
        return self.__measured_cp_data

    def add_measured_cp(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'measured_cp'):
            raise RuntimeWarning('measured_cp already exists')
        # create the subscriber and keep in list
        self.__measured_cp_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_cp',
                                                         geometry_msgs.msg.TransformStamped,
                                                         self.__measured_cp_cb)
        self.__subscribers.append(self.__measured_cp_subscriber)
        # add attributes to class instance
        class_instance.measured_cp = self.__measured_cp


    # internal methods for measured_cv
    def __measured_cv_cb(self, msg):
        self.__measured_cv_data[0] = msg.twist.linear.x
        self.__measured_cv_data[1] = msg.twist.linear.y
        self.__measured_cv_data[2] = msg.twist.linear.z
        self.__measured_cv_data[3] = msg.twist.angular.x
        self.__measured_cv_data[4] = msg.twist.angular.y
        self.__measured_cv_data[5] = msg.twist.angular.z

    def __measured_cv(self):
        return self.__measured_cv_data

    def add_measured_cv(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'measured_cv'):
            raise RuntimeWarning('measured_cv already exists')
        # create the subscriber and keep in list
        self.__measured_cv_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_cv',
                                                         geometry_msgs.msg.TwistStamped,
                                                         self.__measured_cv_cb)
        self.__subscribers.append(self.__measured_cv_subscriber)
        # add attributes to class instance
        class_instance.measured_cv = self.__measured_cv


    # internal methods for measured_cf
    def __measured_cf_cb(self, msg):
        self.__measured_cf_data[0] = msg.wrench.force.x
        self.__measured_cf_data[1] = msg.wrench.force.y
        self.__measured_cf_data[2] = msg.wrench.force.z
        self.__measured_cf_data[3] = msg.wrench.torque.x
        self.__measured_cf_data[4] = msg.wrench.torque.y
        self.__measured_cf_data[5] = msg.wrench.torque.z

    def __measured_cf(self):
        return self.__measured_cf_data

    def add_measured_cf(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'measured_cf'):
            raise RuntimeWarning('measured_cf already exists')
        # create the subscriber and keep in list
        self.__measured_cf_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_cf',
                                                         geometry_msgs.msg.TwistStamped,
                                                         self.__measured_cf_cb)
        self.__subscribers.append(self.__measured_cf_subscriber)
        # add attributes to class instance
        class_instance.measured_cf = self.__measured_cf


    # internal methods for servo_jp
    def __servo_jp(self, setpoint):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.position[:] = setpoint.flat
        self.__servo_jp_publisher.publish(msg)

    def add_servo_jp(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'servo_jp'):
            raise RuntimeWarning('servo_jp already exists')
        # create the subscriber and keep in list
        self.__servo_jp_publisher = rospy.Publisher(self.__ros_namespace + '/servo_jp',
                                                    sensor_msgs.msg.JointState,
                                                    latch = True, queue_size = 1)
        self.__publishers.append(self.__servo_jp_publisher)
        # add attributes to class instance
        class_instance.servo_jp = self.__servo_jp

    # internal methods for servo_cp
    def __servo_cp(self, setpoint):
        # convert to ROS msg and publish
        msg = TransformToMsg(setpoint)
        self.__servo_cp_publisher.publish(msg)

    def add_servo_cp(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'servo_cp'):
            raise RuntimeWarning('servo_cp already exists')
        # create the subscriber and keep in list
        self.__servo_cp_publisher = rospy.Publisher(self.__ros_namespace + '/servo_cp',
                                                    geometry_msgs.msg.TransformStamped,
                                                    latch = True, queue_size = 1)
        self.__publishers.append(self.__servo_cp_publisher)
        # add attributes to class instance
        class_instance.servo_cp = self.__servo_cp

    # internal methods for servo_cf
    def __servo_cf(self, setpoint):
        # convert to ROS msg and publish
        msg = geometry_msgs.msg.WrenchStamped()
        msg.wrench.force.x = setpoint[0]
        msg.wrench.force.y = setpoint[1]
        msg.wrench.force.z = setpoint[2]
        msg.wrench.torque.x = setpoint[3]
        msg.wrench.torque.y = setpoint[4]
        msg.wrench.torque.z = setpoint[5]
        self.__servo_cf_publisher.publish(msg)

    def add_servo_cf(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'servo_cf'):
            raise RuntimeWarning('servo_cf already exists')
        # create the subscriber and keep in list
        self.__servo_cf_publisher = rospy.Publisher(self.__ros_namespace + '/servo_cf',
                                                    geometry_msgs.msg.WrenchStamped,
                                                    latch = True, queue_size = 1)
        self.__publishers.append(self.__servo_cf_publisher)
        # add attributes to class instance
        class_instance.servo_cf = self.__servo_cf


    # internal methods for move_cp
    def __move_cp(self, goal):
        # convert to ROS msg and publish
        msg = TransformToMsg(goal)
        self.__move_cp_publisher.publish(msg)

    def add_move_cp(self, class_instance):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(class_instance, 'move_cp'):
            raise RuntimeWarning('move_cp already exists')
        # create the subscriber and keep in list
        self.__move_cp_publisher = rospy.Publisher(self.__ros_namespace + '/move_cp',
                                                    geometry_msgs.msg.TransformStamped,
                                                    latch = True, queue_size = 1)
        self.__publishers.append(self.__move_cp_publisher)
        # add attributes to class instance
        class_instance.move_cp = self.__move_cp


#     def __init_arm(self, arm_name, ros_namespace = '/dvrk/'):
#         self.__goal_reached = False
#         self.__goal_reached_event = threading.Event()

#         # continuous publish from dvrk_bridge
#         self.__jacobian_spatial = numpy.ndarray(0, dtype = numpy.float)
#         self.__jacobian_body = numpy.ndarray(0, dtype = numpy.float)

#         self.__servo_cf_orientation_absolute_pub = rospy.Publisher(self.__full_ros_namespace
#                                                                    + '/set_wrench_body_orientation_absolute',
#                                                                    Bool, latch = True, queue_size = 1)
#         self.__servo_cf_spatial_pub = rospy.Publisher(self.__full_ros_namespace
#                                                       + '/servo_cf',
#                                                       WrenchStamped, latch = True, queue_size = 1)
#         self.__set_gravity_compensation_pub = rospy.Publisher(self.__full_ros_namespace
#                                                               + '/set_gravity_compensation',
#                                                               Bool, latch = True, queue_size = 1)

#     def __goal_reached_cb(self, data):
#         """Callback for the goal reached.

#         :param data: the goal reached"""
#         self.__goal_reached = data.data
#         self.__goal_reached_event.set()

#     def __jacobian_spatial_cb(self, data):
#         """Callback for the Jacobian in spatial frame.

#         :param data: Jacobian."""
#         jacobian = numpy.asarray(data.data)
#         jacobian.shape = data.layout.dim[0].size, data.layout.dim[1].size
#         self.__jacobian_spatial = jacobian

#     def __jacobian_body_cb(self, data):
#         """Callback for the Jacobian in spatial frame.

#         :param data: Jacobian."""
#         jacobian = numpy.asarray(data.data)
#         jacobian.shape = data.layout.dim[0].size, data.layout.dim[1].size
#         self.__jacobian_body = jacobian

#     def get_arm_current_state(self):
#         """Get the arm current state.
#         :returns: the arm current state
#         :rtype: string"""
#         return self.__arm_current_state


#     def get_arm_desired_state(self):
#         """Get the arm desired state.
#         :returns: the arm desired state
#         :rtype: string"""
#         return self.__arm_desired_state

#     def get_jacobian_spatial(self):
#         """Get the :ref:`jacobian spatial` of the arm.

#         :returns: the jacobian spatial of the arm
#         :rtype: `numpy.ndarray <https://docs.scipy.org/doc/numpy/reference/generated/numpy.ndarray.html>`_"""
#         return self.__jacobian_spatial

#     def get_jacobian_body(self):
#         """Get the :ref:`jacobian body` of the arm.

#         :returns: the jacobian body of the arm
#         :rtype: `numpy.ndarray <https://docs.scipy.org/doc/numpy/reference/generated/numpy.ndarray.html>`_"""
#         return self.__jacobian_body

#     def get_joint_number(self):
#         """Get the number of joints on the arm specified.

#         :returns: the number of joints on the specified arm
#         :rtype: int"""
#         joint_num = len(self.__setpoint_jp)
#         return joint_num

#     def dmove(self, delta_input, interpolate = True, blocking = True):
#         """Incremental motion in cartesian space.

#         :param delta_input: the incremental motion you want to make
#         :param interpolate: see  :ref:`interpolate <interpolate>`
#         """
#         # is this a legal translation input
#         if (self.__check_input_type(delta_input, [PyKDL.Vector, PyKDL.Rotation, PyKDL.Frame])):
#             if (type(delta_input) is PyKDL.Vector):
#                 return self.__dmove_translation(delta_input, interpolate, blocking)
#             elif (type(delta_input) is PyKDL.Rotation):
#                 return self.__dmove_rotation(delta_input, interpolate, blocking)
#             elif (type(delta_input) is PyKDL.Frame):
#                 return self.__dmove_frame(delta_input, interpolate, blocking)


#     def __dmove_translation(self, delta_translation, interpolate = True, blocking = True):
#         """Incremental translation (using PYKDL Vector) in cartesian space.

#         :param delta_translation: the incremental translation you want to make based on the current position, this is in terms of a  `PyKDL.Vector <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
#         :param interpolate: see  :ref:`interpolate <interpolate>`"""
#         # convert into a Frame
#         delta_rotation = PyKDL.Rotation.Identity()
#         delta_frame = PyKDL.Frame(delta_rotation, delta_translation)
#         return self.__dmove_frame(delta_frame, interpolate, blocking)


#     def __dmove_rotation(self, delta_rotation, interpolate = True, blocking = True):
#         """Incremental rotation (using PyKDL Rotation) in cartesian plane.

#         :param delta_rotation: the incremental `PyKDL.Rotation <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_ based upon the current position
#         :param interpolate: see  :ref:`interpolate <interpolate>`"""
#         # convert into a Frame
#         delta_vector = PyKDL.Vector(0.0, 0.0, 0.0)
#         delta_frame = PyKDL.Frame(delta_rotation, delta_vector)
#         return self.__dmove_frame(delta_frame, interpolate, blocking)


#     def __dmove_frame(self, delta_frame, interpolate = True, blocking = True):
#         """Incremental move (using PyKDL Frame) in cartesian plane.

#         :param delta_frame: the incremental `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_ based upon the current position
#         :param interpolate: see  :ref:`interpolate <interpolate>`"""
#         # add the incremental move to the current position, to get the ending frame
#         end_frame = delta_frame * self.__setpoint_cp
#         return self.__move_frame(end_frame, interpolate, blocking)


#     def __move_cartesian_goal(self, end_frame, blocking):
#         """Move the arm to the end position by providing a goal for trajectory generator.

#         :param end_frame: the ending `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
#         :returns: true if you had succesfully move
#         :rtype: Bool"""
#         # set in position cartesian mode
#         end_position= TransformToMsg(end_frame)
#         # go to that position by goal
#         if blocking:
#             return self.__set_position_goal_cartesian_publish_and_wait(end_position)
#         else:
#             self.__move_cp_pub.publish(end_position)
#         return True


#     def __set_position_goal_cartesian_publish_and_wait(self, end_position):
#         """Wrapper around publisher/subscriber to manage events for cartesian coordinates.

#         :param end_position: the ending `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
#         :returns: returns true if the goal is reached
#         :rtype: Bool"""
#         self.__goal_reached_event.clear()
#         # the goal is originally not reached
#         self.__goal_reached = False
#         # recursively call this function until end is reached
#         self.__move_cp_pub.publish(end_position)
#         self.__goal_reached_event.wait(20) # 1 minute at most
#         if not self.__goal_reached:
#             return False
#         return True


#     def dmove_joint(self, delta_pos, interpolate = True, blocking = True):
#         """Incremental move in joint space.

#         :param delta_pos: the incremental amount in which you want to move index by, this is in terms of a numpy array
#         :param interpolate: see  :ref:`interpolate <interpolate>`"""
#         if ((not(type(delta_pos) is numpy.ndarray))
#              or (not(delta_pos.dtype == numpy.float64))):
#             print "delta_pos must be an array of floats"
#             return False
#         if (not(delta_pos.size ==  self.get_joint_number())):
#             print "delta_pos must be an array of size", self.get_joint_number()
#             return False

#         abs_pos = numpy.array(self.__setpoint_jp)
#         abs_pos = abs_pos+ delta_pos
#         return self.__move_joint(abs_pos, interpolate, blocking)


#     def dmove_joint_one(self, delta_pos, indices, interpolate = True, blocking = True):
#         """Incremental index move of 1 joint in joint space.

#         :param delta_pos: the incremental amount in which you want to move index by, this is a float
#         :param index: the joint you want to move, this is an integer
#         :param interpolate: see  :ref:`interpolate <interpolate>`"""
#         if (type(delta_pos) is float and type(indices) is int):
#             return self.dmove_joint_some(numpy.array([delta_pos]), numpy.array([indices]), interpolate, blocking)
#         else:
#             return False


#     def dmove_joint_some(self, delta_pos, indices, interpolate = True, blocking = True):
#         """Incremental index move of a series of joints in joint space.

#         :param delta_pos: the incremental amount in which you want to move index by, this is a numpy array corresponding to the number of indices
#         :param indices: the joints you want to move, this is a numpy array of indices
#         :param interpolate: see  :ref:`interpolate <interpolate>`"""

#         # check if delta is an array
#         if ((not(type(delta_pos) is numpy.ndarray))
#              or (not(delta_pos.dtype == numpy.float64))):
#             print "delta_pos must be an array of floats"
#             return False

#         # check the length of the delta move
#         if ((not(type(indices) is numpy.ndarray))
#             or (not(indices.dtype == numpy.int64))):
#             print "indices must be an array of integers"
#             return False

#         if ((not(indices.size == delta_pos.size))
#             or (indices.size > self.get_joint_number())):
#             print "size of delta_pos and indices must match and be less than", self.get_joint_number()
#             return False

#         for i in range(indices.size):
#             if (indices[i] > self.get_joint_number()):
#                 print "all indices must be less than", self.get_joint_number()
#                 return False

#         abs_pos = numpy.array(self.__setpoint_jp)
#         for i in range(len(indices)):
#             abs_pos[indices[i]] = abs_pos[indices[i]] + delta_pos[i]

#         # move accordingly
#         return self.__move_joint(abs_pos, interpolate, blocking)


#     def move_joint(self, abs_pos, interpolate = True, blocking = True):
#         """Absolute move in joint space.

#         :param abs_pos: the absolute position in which you want to move, this is a numpy array
#         :param interpolate: see  :ref:`interpolate <interpolate>`"""

#         if ((not(type(abs_pos) is numpy.ndarray))
#             or (not(abs_pos.dtype == numpy.float64))):
#             print "abs_pos must be an array of floats"
#             return False
#         if (not(abs_pos.size == self.get_joint_number())):
#             print "abs_pos must be an array of size", self.get_joint_number()
#             return False

#         return self.__move_joint(abs_pos, interpolate, blocking)


#     def move_joint_one(self, abs_pos, joint_index, interpolate = True, blocking = True):
#         """Absolute index move of 1 joint in joint space.

#         :param value: the absolute amount in which you want to move index by, this is a list
#         :param index: the joint you want to move, this is a list
#         :param interpolate: see  :ref:`interpolate <interpolate>`"""
#         if ((type(abs_pos) is float) and (type(joint_index) is int)):
#             return self.move_joint_some(numpy.array([abs_pos]), numpy.array([joint_index]), interpolate, blocking)
#         else:
#             return False


#     def move_joint_some(self, abs_pos, indices, interpolate = True, blocking = True):
#         """Absolute index move of a series of joints in joint space.

#         :param value: the absolute amount in which you want to move index by, this is a list
#         :param index: the joint you want to move, this is a list
#         :param interpolate: see  :ref:`interpolate <interpolate>`"""

#         if ((not(type(abs_pos) is numpy.ndarray))
#             or (not(abs_pos.dtype == numpy.float64))):
#             print "delta_pos must be an array of floats"
#             return False

#         # check the length of the delta move
#         if ((not(type(indices) is numpy.ndarray))
#             or (not(indices.dtype == numpy.int64))):
#             print "indices must be an array of integers"
#             return False

#         if ((not(indices.size == abs_pos.size))
#             or (indices.size > self.get_joint_number())):
#             print "size of delta_pos and indices must match and be less than", self.get_joint_number()
#             return False

#         for i in range(indices.size):
#             if (indices[i] > self.get_joint_number()):
#                 print "all indices must be less than", self.get_joint_number()
#                 return False

#         abs_pos_result = numpy.array(self.__setpoint_jp)
#         for i in range(len(indices)):
#             abs_pos_result[indices[i]] = abs_pos[i]

#         # move accordingly
#         return self.__move_joint(abs_pos_result, interpolate, blocking)


#     def __move_joint(self, abs_joint, interpolate = True, blocking = True):
#         """Absolute move by vector in joint plane.

#         :param abs_joint: the absolute position of the joints in terms of a numpy array
#         :param interpolate: if false the trajectory generator will be used; if true you can bypass the trajectory generator"""
#         if (interpolate):
#             return self.__move_joint_goal(abs_joint, blocking)
#         else:
#             return self.__move_joint_direct(abs_joint)



#     def __move_joint_goal(self, end_joint, blocking):
#         """Move the arm to the end vector by bypassing the trajectory generator.

#         :param end_joint: the list of joints in which you should conclude movement
#         :returns: true if you had succesfully move
#         :rtype: Bool"""
#         joint_state = JointState()
#         joint_state.position[:] = end_joint.flat
#         if blocking:
#             return self.__set_position_goal_joint_publish_and_wait(joint_state)
#         else:
#             self.__move_jp_pub.publish(joint_state)
#         return True


#     def __set_position_goal_joint_publish_and_wait(self, end_position):
#         """Wrapper around publisher/subscriber to manage events for joint coordinates.

#         :param end_position: there is only one parameter, end_position which tells us what the ending position is
#         :returns: whether or not you have successfully moved by goal or not
#         :rtype: Bool"""
#         self.__goal_reached_event.clear()
#         self.__goal_reached = False
#         self.__move_jp_pub.publish(end_position)
#         self.__goal_reached_event.wait(20) # 1 minute at most
#         if not self.__goal_reached:
#             return False
#         return True


#     def servo_jf(self, effort):
#         if ((not(type(effort) is numpy.ndarray))
#             or (not(effort.dtype == numpy.float64))):
#             print "effort must be an array of floats"
#             return False
#         if (not(effort.size == self.get_joint_number())):
#             print "effort must be an array of size", self.get_joint_number()
#             return False
#         joint_state = JointState()
#         joint_state.effort[:] = effort.flat
#         self.__servo_jf_pub.publish(joint_state)
#         return True


#     def servo_cf_spatial(self, force):
#         """Apply a wrench with force only (spatial), torque is null

#         :param force: the new force to set it to
#         """
#         w = WrenchStamped()
#         w.wrench.force.x = force[0]
#         w.wrench.force.y = force[1]
#         w.wrench.force.z = force[2]
#         w.wrench.torque.x = 0.0
#         w.wrench.torque.y = 0.0
#         w.wrench.torque.z = 0.0
#         self.__servo_cf_spatial_pub.publish(w)


#     def servo_cf_body_orientation_absolute(self, absolute):
#         """Apply body wrench using body orientation (relative/False) or reference frame (absolute/True)"""
#         m = Bool()
#         m.data = absolute
#         self.__servo_cf_orientation_absolute_pub.publish(m)


#     def set_gravity_compensation(self, gravity_compensation):
#         "Turn on/off gravity compensation in cartesian effort mode"
#         g = Bool()
#         g.data = gravity_compensation
#         self.__set_gravity_compensation_pub.publish(g)
