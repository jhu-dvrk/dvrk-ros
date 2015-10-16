"""This class presents a robot api for the da Vinci Research Kit.
Remember that for this program to work, you will need to import the
robot class, this can be done by `import robot` an well as initialize
the robot. For example, if we want to create a robot called `r`, for
robot `PSM1`, we will simply type `r = robot.robot('PSM1')` in iPython
and `r = robot('PSM1')` in python.

.. _interpolate:

Interpolation
=============

If the `interpolation` flag is set to `True` (default), the robot
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

The robot controller can provide two different positions at any given
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
`forces (gravity, friction, ...) <https://en.wikipedia.org/wiki/Force>`_ applied on the robot.  When
implementing an incremental move, one should always use the last
desired position.  If one needs to track the robot, it is better to
use the current position.

Robot API
=========

"""

# sphinx-apidoc -F -A "Yijun Hu" -o doc src

import rospy
import threading
import math
import sys
import logging
import time
import inspect
import code
import IPython
import math

from PyKDL import *
from tf import transformations
from tf_conversions import posemath
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState

from code import InteractiveConsole
from imp import new_module

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

class robot:
    """Simple robot API wrapping around ROS messages
    """

    # initialize the robot
    def __init__(self, robot_name, ros_namespace = '/dvrk/'):
        """Constructor.  This initializes a few data members.It
        requires a robot name, this will be used to find the ROS
        topics for the robot being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `/dvrk/PSM1`"""
        # data members, event based
        self.__robot_name = robot_name
        self.__ros_namespace = ros_namespace
        self.__robot_state = 'uninitialized'
        self.__robot_state_event = threading.Event()
        self.__goal_reached = False
        self.__goal_reached_event = threading.Event()

        # continuous publish from dvrk_bridge
        self.__position_joint_desired = []
        self.__effort_joint_desired = []
        self.__position_cartesian_desired = Frame()
        self.__position_joint_current = []
        self.__velocity_joint_current = []
        self.__effort_joint_current = []
        self.__position_cartesian_current = Frame()

        # publishers
        frame = Frame()
        full_ros_namespace = self.__ros_namespace + self.__robot_name
        self.set_robot_state = rospy.Publisher(full_ros_namespace + '/set_robot_state',
                                               String, latch=True, queue_size=1)
        self.set_position_joint = rospy.Publisher(full_ros_namespace + '/set_position_joint',
                                                  JointState, latch=True, queue_size=1)
        self.set_position_goal_joint = rospy.Publisher(full_ros_namespace + '/set_position_goal_joint',
                                                       JointState, latch=True, queue_size=1)
        self.set_position_cartesian = rospy.Publisher(full_ros_namespace + '/set_position_cartesian',
                                                      Pose, latch=True, queue_size=1)
        self.set_position_goal_cartesian = rospy.Publisher(full_ros_namespace + '/set_position_goal_cartesian',
                                                           Pose, latch=True, queue_size=1)
        self.set_jaw_position = rospy.Publisher(full_ros_namespace + '/set_jaw_position',
                                                Float32, latch=True, queue_size=1)

        # subscribers
        rospy.Subscriber(full_ros_namespace + '/robot_state',
                         String, self.__robot_state_callback)
        rospy.Subscriber(full_ros_namespace + '/goal_reached',
                         Bool, self.__goal_reached_callback)
        rospy.Subscriber(full_ros_namespace + '/state_joint_desired',
                         JointState, self.__state_joint_desired_callback)
        rospy.Subscriber(full_ros_namespace + '/position_cartesian_desired',
                         Pose, self.__position_cartesian_desired_callback)
        rospy.Subscriber(full_ros_namespace + '/state_joint_current',
                         JointState, self.__state_joint_current_callback)
        rospy.Subscriber(full_ros_namespace + '/position_cartesian_current',
                         Pose, self.__position_cartesian_current_callback)
        # create node
        # rospy.init_node('robot_api', anonymous = True)
        rospy.init_node('robot_api',anonymous = True, log_level = rospy.WARN)
        rospy.loginfo(rospy.get_caller_id() + ' -> started robot: ' + self.__robot_name)

    def __robot_state_callback(self, data):
        """Callback for robot state.

        :param data: the current robot state"""
        rospy.loginfo(rospy.get_caller_id() + " -> current state is %s", data.data)
        self.__robot_state = data.data
        self.__robot_state_event.set()

    def __goal_reached_callback(self, data):
        """Callback for the goal reached.

        :param data: the goal reached"""
        rospy.loginfo(rospy.get_caller_id() + " -> goal reached is %s", data.data)
        self.__goal_reached = data.data
        self.__goal_reached_event.set()

    def __state_joint_desired_callback(self, data):
        """Callback for the joint desired position.

        :param data: the `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_desired"""
        self.__position_joint_desired[:] = data.position
        self.__effort_joint_desired[:] = data.effort

    def __position_cartesian_desired_callback(self, data):
        """Callback for the cartesian desired position.

        :param data: the cartesian position desired"""
        self.__position_cartesian_desired = posemath.fromMsg(data)

    def __state_joint_current_callback(self, data):
        """Callback for the current joint position.

        :param data: the `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_current"""
        self.__position_joint_current[:] = data.position
        self.__velocity_joint_current[:] = data.velocity
        self.__effort_joint_current[:] = data.effort

    def __position_cartesian_current_callback(self, data):
        """Callback for the current cartesian position.

        :param data: The cartesian position current."""
        self.__position_cartesian_current = posemath.fromMsg(data)

    def __dvrk_set_state(self, state, timeout = 5):
        """Simple set state with block.

        :param state: the robot state
        :param timeout: the lenghth you want to wait for robot to change state
        :return: whether or not the robot state has been successfuly set
        :rtype: Bool"""
        if (self.__robot_state == state):
            return True
        self.__robot_state_event.clear()
        self.set_robot_state.publish(state)
        self.__robot_state_event.wait(timeout)
        # if the state is not changed return False
        if (self.__robot_state != state):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state ' + state)
            return False
        return True

    def home(self):
        """This method will provide power to the robot as will as home
        the robot. This method requries the robot name."""
        rospy.loginfo(rospy.get_caller_id() + ' -> start homing')
        self.__robot_state_event.clear()
        self.set_robot_state.publish('Home')
        counter = 10 # up to 10 transitions to get ready
        while (counter > 0):
            self.__robot_state_event.wait(20) # give up to 20 secs for each transition
            if (self.__robot_state != 'DVRK_READY'):
                self.__robot_state_event.clear()
                counter = counter - 1
                rospy.loginfo(rospy.get_caller_id() + ' -> waiting for state to be DVRK_READY')
            else:
                counter = -1
        if (self.__robot_state != 'DVRK_READY'):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state DVRK_READY')
        rospy.loginfo(rospy.get_caller_id() + ' <- homing complete')

    def shutdown(self):
        """Stops providing power to the robot."""
        rospy.loginfo(rospy.get_caller_id() + ' -> end homing')
        self.__dvrk_set_state('DVRK_UNINITIALIZED', 20)

    def get_robot_state(self):
        return self.__robot_state

    def get_current_cartesian_position(self):
        """Gets the :ref:`current cartesian position <currentvdesired>` of the robot in terms of cartesian space.

        :returns: the current position of the robot in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__position_cartesian_current

    def get_current_joint_position(self):
        """Gets the :ref:`current joint position <currentvdesired>` of the robot in terms of joint space.

        :returns: the current position of the robot in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__position_joint_current

    def get_current_joint_velocity(self):
        """Gets the :ref:`current joint velocity <currentvdesired>` of the robot in terms of joint space.

        :returns: the current position of the robot in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__velocity_joint_current

    def get_current_joint_effort(self):
        """Gets the :ref:`current joint effort <currentvdesired>` of the robot in terms of joint space.

        :returns: the current position of the robot in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__effort_joint_current

    def get_desired_cartesian_position(self):
        """Get the :ref:`desired cartesian position <currentvdesired>` of the robot in terms of caretsian space.

        :returns: the desired position of the robot in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__position_cartesian_desired

    def get_desired_joint_position(self):
        """Gets the :ref:`desired joint position <currentvdesired>` of the robot in terms of joint space.

        :returns: the desired position of the robot in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__position_joint_desired

    def get_desired_joint_effort(self):
        """Gets the :ref:`desired joint effort <currentvdesired>` of the robot in terms of joint space.

        :returns: the desired effort of the robot in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__effort_joint_desired

    def get_joint_number(self):
        """Gets the number of joints on the arm specified.

        :returns: the number of joints on the specified arm
        :rtype: int"""
        joint_num = len(self.__position_joint_desired)
        return joint_num

    def __check_input_type(self, input, type_list):
        """check if the data input is a data type that is located in type_list

        :param input: The data type that needs to be checked.
        :param type_list : A list of types to check input against.
        :returns: whether or not the input is a type in type_list
        :rtype: Bool"""
        found = False
        # check the input against all input_type
        for i in range (len(type_list)):
            if (type(input) is type_list[i]):
                if(not(type(input) is list)):
                    return True
                else:
                    found = True
                    found1 = True
                    # if the list is of type list, check that each input is of
                    # the type that is after list in type_list
                    for j in range(len(input)):
                        if (not (type(input[j]) is type_list[i+1])):
                            found1 = False
                        else:
                            i+1
                    # print statements for error inside list
                    if(found1 == False):
                        print 'Error in ', inspect.stack()[1][3], 'list should be made up of', type_list[i+1],'and not of'
                        print_type1 = ' '
                        for k in range(len(input)):
                            print_medium = ' ' + str(type(input[k]))
                            print_type1 += print_medium
                        print print_type1
                    else:
                        return True
        # not of type_list print state for this error inside
        if (found == False):
            print 'Error in ', inspect.stack()[1][3], 'input is of type', input, 'and is not one of:'
            print_type2 = ''
            # skip_length
            i = 0
            while i < len(type_list):
                print_medium2 = ' '+ str(type_list[i])
                print_type2 += print_medium2
                if(type_list[i] == list):
                    i+=1
                i+=1
            print print_type2
        return False

    def __check_list_length(self, check_list, check_length):
        """check that the list is of desired length

        :param list: the list you want to check
        :param check_length: the integer to check it against
        :returns: whether or not the length of check_list is equal to check_length
        :rtype: Bool"""
        if (len(check_list) == check_length):
            return True
        else:
            print 'input is of size', len(check_list), 'but required size is', check_length
            # sperspace = new_module('superspace')
            # sperspace.check_list = check_list
            # console = Console({'superspace': superspace})
            # console.interact()
            # print 'new value of list ', superspace.check_list
            # check_list[:] = superspace.check_list
            # print 'check_list', check_list
            return False ####   should be False or actually based on user's return code from console

    def close_gripper(self):
        "Close the arm gripper"
        if (not self.__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        self.set_jaw_position.publish(-10.0 * math.pi / 180.0);

    def open_gripper(self):
        "Open the arm gripper"
        if (not self.__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        self.set_jaw_position.publish(80.0 * math.pi / 180.0);

    def delta_move_cartesian(self, delta_input, interpolate=True):
        """Incremental translation in cartesian space.

        :param delta_input: the incremental translation you want to make
        :param interpolate: see  :ref:`interpolate <interpolate>`
        """
        rospy.loginfo(rospy.get_caller_id() + ' -> starting delta move cartesian translation')
        # is this a legal translation input
        if(self.__check_input_type(delta_input, [list, float, Vector, Rotation, Frame])):
               if(self.__check_input_type(delta_input, [list, float, Vector])):
                   self.delta_move_cartesian_translation(delta_input, interpolate)
               elif(self.__check_input_type(delta_input, [Rotation])):
                   self.delta_move_cartesian_rotation(delta_input, interpolate)
               elif(self.__check_input_type(delta_input, [Frame])):
                   self.delta_move_cartesian_frame(delta_input, interpolate)
        rospy.loginfo(rospy.get_caller_id() + ' -> completing delta move cartesian translation')

    def delta_move_cartesian_translation(self, delta_translation, interpolate=True):
        """Incremental translation in cartesian space.

        :param delta_translation: the incremental translation you want to make based on the current position, this is in terms of a  `PyKDL.Vector <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_ or a list of floats of size 3
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting delta move cartesian translation')
        # is this a legal translation input
        if(self.__check_input_type(delta_translation, [list, float,Vector])):
            if(type(delta_translation) is list):
                if (self.__check_list_length(delta_translation, 3)):
                    # convert into a Vector
                    delta_vector = Vector(delta_translation[0], delta_translation[1], delta_translation[2])
                else:
                    return
            else:
                delta_vector = delta_translation
            # convert into a Frame
            delta_rotation = Rotation.Identity()
            delta_frame = Frame(delta_rotation, delta_vector)
            # move accordingly
            self.delta_move_cartesian_frame(delta_frame, interpolate)
            rospy.loginfo(rospy.get_caller_id() + ' -> completing delta move cartesian translation')

    def delta_move_cartesian_rotation(self, delta_rotation, interpolate=True):
        """Incremental rotation in cartesian plane.

        :param delta_rotation: the incremental `PyKDL.Rotation <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_ based upon the current position
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting delta move cartesian rotation')
        # is this a legal rotation input
        if(self.__check_input_type(delta_rotation, [Rotation])):
            # convert into a Frame
            delta_vector = Vector(0.0, 0.0, 0.0)
            delta_frame = Frame(delta_rotation, delta_vector)
            # move accordingly
            self.delta_move_cartesian_frame(delta_frame, interpolate)
            rospy.loginfo(rospy.get_caller_id() + ' -> completing delta move cartesian rotation')

    def delta_move_cartesian_frame(self, delta_frame, interpolate=True):
        """Incremental move by Frame in cartesian plane.

        :param delta_frame: the incremental `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_ based upon the current position
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting delta move cartesian frame')
        # is this a legal frame input
        if (self.__check_input_type(delta_frame, [Frame])):
            # add the incremental move to the current position, to get the ending frame
            end_frame = delta_frame * self.__position_cartesian_desired
            # move accordingly
            self.move_cartesian_frame(end_frame, interpolate)
            rospy.loginfo(rospy.get_caller_id() + ' -> completing delta move cartesian frame')


    def move_cartesian_translation(self, abs_translation, interpolate=True):
        """Absolute translation in cartesian space.

        :param abs_translation: the absolute translation you want to make, this is in terms of a  `PyKDL.Vector <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_ or a list of floats of size 3
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting absolute move cartesian translation')
        # is this a legal translation input
        if(self.__check_input_type(abs_translation, [list,float,Vector])):
            # if the input is a list convert it into a vector
            if(type(abs_translation) is list):
                if (self.__check_list_length(abs_translation, 3)):
                    # convert intoa vector
                    abs_vector = Vector(abs_translation[0], abs_translation[1], abs_translation[2])
                else:
                    return
            else:
                abs_vector = abs_translation
            # convert into a Frame
            abs_rotation = self.__position_cartesian_desired.M
            abs_frame = Frame(abs_rotation, abs_vector)
            # move accordingly
            self.move_cartesian_frame(abs_frame, interpolate)
            rospy.loginfo(rospy.get_caller_id() + ' -> completing absolute move cartesian translation')

    def move_cartesian(self, abs_input, interpolate=True):
        """Absolute translation in cartesian space.

        :param abs_input: the absolute translation you want to make
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting absolute move cartesian translation')
        # is this a legal translation input
        if(self.__check_input_type(abs_input, [list, float, Vector, Rotation, Frame])):
               if(self.__check_input_type(abs_input, [list, float, Vector])):
                   self.move_cartesian_translation(abs_input, interpolate)
               elif(self.__check_input_type(abs_input, [Rotation])):
                   self.move_cartesian_rotation(abs_input, interpolate)
               elif(self.__check_input_type(abs_input, [Frame])):
                   self.move_cartesian_frame(abs_input, interpolate)
        rospy.loginfo(rospy.get_caller_id() + ' -> completing absolute move cartesian translation')

    def move_cartesian_rotation(self, abs_rotation, interpolate=True):
        """Absolute rotation in cartesian plane.

        :param abs_rotation: the absolute `PyKDL.Rotation <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting absolute move cartesian rotation')
        # is this a legal rotation input
        if(self.__check_input_type(abs_rotation, [Rotation])):
            # convert into a Frame
            abs_vector = self.__position_cartesian_desired.p
            abs_frame = Frame(abs_rotation, abs_vector)
            # move accordingly
            self.move_cartesian_frame(abs_frame, interpolate)
            rospy.loginfo(rospy.get_caller_id() + ' -> completing absolute move cartesian rotation')


    def move_cartesian_frame(self, abs_frame, interpolate=True):
        """Absolute move by Frame in cartesian plane.

        :param abs_frame: the absolute `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting absolute move cartesian frame')
        if (self.__check_input_type(abs_frame, [Frame])):
            # move based on value of interpolate
            if (interpolate):
                self.__move_cartesian_goal(abs_frame)
            else:
                self.__move_cartesian_direct(abs_frame)
            rospy.loginfo(rospy.get_caller_id() + ' -> completing absolute move cartesian frame')

    def __move_cartesian_direct(self, end_frame):
        """Move the robot to the end position by passing the trajectory generator.

        :param end_frame: the ending `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :returns: true if you had successfully move
        :rtype: Bool"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting move cartesian direct')
        # set in position cartesian mode
        end_position = posemath.toMsg(end_frame)
        if (not self.__dvrk_set_state('DVRK_POSITION_CARTESIAN')):
            return False
        # go to that position directly
        self.set_position_cartesian.publish(end_position)
        rospy.loginfo(rospy.get_caller_id() + ' <- completing move cartesian direct')
        return True

    def __move_cartesian_goal(self, end_frame):
        """Move the robot to the end position by providing a goal for trajectory generator.

        :param end_frame: the ending `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_
        :returns: true if you had succesfully move
        :rtype: Bool"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting move cartesian goal')
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
        self.set_position_goal_cartesian.publish(end_position)
        self.__goal_reached_event.wait(20) # 1 minute at most
        if not self.__goal_reached:
            return False
        rospy.loginfo(rospy.get_caller_id() + ' -> compeleting set position goal cartesian publish and wait')
        return True

    def delta_move_joint_list(self, value, index=[], interpolate=True):
        """Incremental index move in joint space.

        :param value: the incremental amount in which you want to move index by, this is a list
        :param index: the joint you want to move, this is a list
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting delta move joint index')
        # check if value is a list
        if(self.__check_input_type(value, [list,float])):
            initial_joint_position = self.__position_joint_desired
            delta_joint = []
            delta_joint[:] = initial_joint_position
            # give index is not given and the size of the value is 7
            if (index == []):
                if(self.__check_list_length(value, len(self.__position_joint_desired))):
                    index = range(len(self.__position_joint_desired))
            # is there both an index and a value
            else:
                # check the length of the delta move
                if(self.__check_input_type(index, [list,int]) and len(index) == len(value)):
                    # make sure it does not exceed the legal joint amount
                    if(len(index) <= len(initial_joint_position)):
                        for j in range(len(index)):
                            if(index[j] < len(initial_joint_position)):
                                for i in range (len(initial_joint_position)):
                                    if i == index[j]:
                                        delta_joint[i] = initial_joint_position[i] + value[j]
                    # move accordingly
                    self.__move_joint(delta_joint, interpolate)
                else:
                    return

    def move_joint_list(self, value, index = [], interpolate=True):
        """Absolute index move in joint space.

        :param value: the incremental amount in which you want to move index by, this is a list
        :param index: the incremental joint you want to move, this is a list
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting abs move joint index')
        # check if value is a list
        if(self.__check_input_type(value, [list,float])):
            initial_joint_position = self.__position_joint_desired
            abs_joint = []
            abs_joint[:] = initial_joint_position
            # give index is not given and the size of the value is 7
            if (index == []):
                if(self.__check_list_length(value, len(self.__position_joint_desired))):
                    index = range(len(self.__position_joint_desired))
            # is there both an index and a value
            if(self.__check_input_type(index, [list,int]) and len(index) == len(value)):
            # if the joint specified exists
                if(len(index) <= len(initial_joint_position)):
                    for j in range(len(index)):
                        if(index[j] < len(initial_joint_position)):
                            for i in range (len(initial_joint_position)):
                                if i == index[j]:
                                    abs_joint[i] = value[j]
                    self.__move_joint(abs_joint, interpolate)

    def __move_joint(self, abs_joint, interpolate = True):
        """Absolute move by vector in joint plane.

        :param abs_joint: the absolute position of the joints in terms of a list
        :param interpolate: if false the trajectory generator will be used; if true you can bypass the trajectory generator"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting absolute move joint vector')
        if(self.__check_input_type(abs_joint, [list,float])):
            if (interpolate):
                self.__move_joint_goal(abs_joint)
            else:
                self.__move_joint_direct(abs_joint)
        rospy.loginfo(rospy.get_caller_id() + ' -> completing absolute move joint vector')

    def __move_joint_direct(self, end_joint):
        """Move the robot to the end vector by passing the trajectory generator.

        :param end_joint: the list of joints in which you should conclude movement
        :returns: true if you had succesfully move
        :rtype: Bool"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting move joint direct')
        if (self.__check_input_type(end_joint, [list,float])):
            if not self.__dvrk_set_state('DVRK_POSITION_JOINT'):
                return False
            # go to that position directly
            joint_state = JointState()
            joint_state.position[:] = end_joint
            self.set_position_joint.publish(joint_state)
            rospy.loginfo(rospy.get_caller_id() + ' <- completing move joint direct')
            return True

    def __move_joint_goal(self, end_joint):
        """Move the robot to the end vector by bypassing the trajectory generator.

        :param end_joint: the list of joints in which you should conclude movement
        :returns: true if you had succesfully move
        :rtype: Bool"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting move joint goal')
        if (self.__check_input_type(end_joint, [list,float])):
            if (not self.__dvrk_set_state('DVRK_POSITION_GOAL_JOINT')):
                return False
            joint_state = JointState()
            joint_state.position[:] = end_joint
            self.__set_position_goal_joint_publish_and_wait(joint_state)
            return True

    def __set_position_goal_joint_publish_and_wait(self, end_position):
        """Wrapper around publisher/subscriber to manage events for joint coordinates.

        :param end_position: there is only one parameter, end_position which tells us what the ending position is
        :returns: whether or not you have successfully moved by goal or not
        :rtype: Bool"""
        self.__goal_reached_event.clear()
        self.__goal_reached = False
        self.set_position_goal_joint.publish(end_position)
        self.__goal_reached_event.wait(20) # 1 minute at most
        if not self.__goal_reached:
            return False
        rospy.loginfo(rospy.get_caller_id() + ' -> completing set position goal joint publish and wait')
        return True
