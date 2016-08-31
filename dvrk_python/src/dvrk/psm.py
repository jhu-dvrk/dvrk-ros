#  Author(s):  Anton Deguet
#  Created on: 2016-05

#   (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

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
        # publishers
        self.__set_jaw_position_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                      + '/set_jaw_position',
                                                      Float32, latch = True, queue_size = 1)
        self.__set_tool_present_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                      + '/set_tool_present',
                                                      Bool, latch = True, queue_size = 1)


    def get_current_jaw_position(self):
        "get the current angle of the jaw"
        return self._arm__position_joint_current[6]


    def get_desired_jaw_position(self):
        "get the desired angle of the jaw"
        return self._arm__position_joint_desired[6]


    def close_jaw(self):
        "Close the tool jaw"
        if (not self._arm__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        return self.__set_jaw_position_pub.publish(-20.0 * math.pi / 180.0)


    def open_jaw(self):
        "Open the tool jaw"
        if (not self._arm__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        return self.__set_jaw_position_pub.publish(80.0 * math.pi / 180.0)


    def move_jaw(self, set_jaw):
        "Set the jaw tool to set_jaw"
        if (not self._arm__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        if ((set_jaw >= -20.0 * math.pi / 180.0) and (set_jaw <= 80.0 * math.pi / 180.0)):
            return self.__set_jaw_position_pub.publish(set_jaw)
        else:
            print 'not a valid jaw position'


    def insert_tool(self, depth):
        "insert the tool, by moving it to an absolute depth"
        return self.move_joint_one(depth, 2)


    def dinsert_tool(self, depth):
        "insert the tool, by moving it an additional depth"
        return self.dmove_joint_one(depth, 2)


    def set_tool_present(self, tool_present):
        "Set tool inserted.  To be used only for custom tools that can't be detected automatically"
        ti = Bool()
        ti.data = tool_present
        self.__set_tool_present_pub.publish(ti)
