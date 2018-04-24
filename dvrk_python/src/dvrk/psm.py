#  Author(s):  Anton Deguet
#  Created on: 2016-05

#   (C) Copyright 2016-2018 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

from dvrk.arm import *

class psm(arm):
    """Simple robot API wrapping around ROS messages
    """
    # initialize the robot
    def __init__(self, psm_name, ros_namespace = '/dvrk/'):
        # first call base class constructor
        self._arm__init_arm(psm_name, ros_namespace)

        # jaw states
        self.__position_jaw_desired = 0.0
        self.__effort_jaw_desired = 0.0
        self.__position_jaw_current = 0.0
        self.__velocity_jaw_current = 0.0
        self.__effort_jaw_current = 0.0

        # publishers
        self.__set_position_jaw_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                      + '/set_position_jaw',
                                                      JointState, latch = True, queue_size = 1)
        self.__set_position_goal_jaw_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                           + '/set_position_goal_jaw',
                                                           JointState, latch = True, queue_size = 1)
        self.__set_effort_jaw_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                    + '/set_effort_jaw',
                                                    JointState, latch = True, queue_size = 1)
        self.__set_tool_present_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                      + '/set_tool_present',
                                                      Bool, latch = True, queue_size = 1)

        self._arm__pub_list.extend([self.__set_position_jaw_pub,
                                    self.__set_position_goal_jaw_pub,
                                    self.__set_effort_jaw_pub,
                                    self.__set_tool_present_pub])
        # subscribers
        self._arm__sub_list.extend([
        rospy.Subscriber(self._arm__full_ros_namespace + '/state_jaw_desired',
                         JointState, self.__state_jaw_desired_cb),
        rospy.Subscriber(self._arm__full_ros_namespace + '/state_jaw_current',
                         JointState, self.__state_jaw_current_cb)])


    def __state_jaw_desired_cb(self, data):
        if (len(data.position) == 1):
            self.__position_jaw_desired = data.position[0]
            self.__effort_jaw_desired = data.effort[0]


    def __state_jaw_current_cb(self, data):
        if (len(data.position) == 1):
            self.__position_jaw_current = data.position[0]
            self.__velocity_jaw_current = data.velocity[0]
            self.__effort_jaw_current = data.effort[0]


    def get_current_jaw_position(self):
        "get the current angle of the jaw"
        return self.__position_jaw_current

    def get_current_jaw_velocity(self):
        "get the current angular velocity of the jaw"
        return self.__velocity_jaw_current

    def get_current_jaw_effort(self):
        "get the current torque applied to the jaw"
        return self.__effort_jaw_current


    def get_desired_jaw_position(self):
        "get the desired angle of the jaw"
        return self.__position_jaw_desired

    def get_desired_jaw_effort(self):
        "get the desired torque to be applied to the jaw"
        return self.__effort_jaw_desired


    def close_jaw(self, interpolate = True, blocking = True):
        "Close the tool jaw"
        return self.move_jaw(-20.0 * math.pi / 180.0, interpolate, blocking)

    def open_jaw(self, interpolate = True, blocking = True):
        "Open the tool jaw"
        return self.move_jaw(80.0 * math.pi / 180.0, interpolate, blocking)

    def move_jaw(self, angle_radian, interpolate = True, blocking = True):
        "Set the jaw tool to set_jaw in radians"
        # create payload
        joint_state = JointState()
        joint_state.position.append(angle_radian)
        # check for interpolation
        if interpolate:
            if blocking:
                self._arm__goal_reached_event.clear()
                self._arm__goal_reached = False
            self.__set_position_goal_jaw_pub.publish(joint_state)
            if blocking:
                self._arm__goal_reached_event.wait(20)
                if not self._arm__goal_reached:
                    return False
            return True
        else:
            return self.__set_position_jaw_pub.publish(joint_state)

    def set_effort_jaw(self, effort):
        # create payload
        joint_state = JointState()
        joint_state.effort.append(effort)
        return self.__set_effort_jaw_pub.publish(joint_state)

    def insert_tool(self, depth, interpolate = True, blocking = True):
        "insert the tool, by moving it to an absolute depth"
        return self.move_joint_one(depth, 2, interpolate, blocking)


    def dinsert_tool(self, depth, interpolate = True, blocking = True):
        "insert the tool, by moving it an additional depth"
        return self.dmove_joint_one(depth, 2, interpolate, blocking)


    def set_tool_present(self, tool_present):
        "Set tool inserted.  To be used only for custom tools that can't be detected automatically"
        ti = Bool()
        ti.data = tool_present
        self.__set_tool_present_pub.publish(ti)
