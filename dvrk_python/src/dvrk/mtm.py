#  Author(s):  Anton Deguet
#  Created on: 2016-05

#   (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

from dvrk.arm import *

class mtm(arm):
    """Simple robot API wrapping around ROS messages
    """
    # initialize the robot
    def __init__(self, mtm_name, ros_namespace = '/dvrk/'):
        # first call base class constructor
        self._arm__init_arm(mtm_name, ros_namespace)

        # gripper states
        self.__position_gripper_current = 0.0

        # publishers
        self.__lock_orientation_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                      + '/lock_orientation',
                                                      Quaternion, latch=True, queue_size = 1)
        self.__unlock_orientation_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                        + '/unlock_orientation',
                                                        Empty, latch=True, queue_size = 1)

        self._arm__pub_list.extend([self.__lock_orientation_pub,
                               self.__unlock_orientation_pub])
        # subscribers
        self._arm__sub_list.extend([rospy.Subscriber(self._arm__full_ros_namespace + '/state_gripper_current',
                         JointState, self.__state_gripper_current_cb)])


    def __state_gripper_current_cb(self, data):
        self.__position_gripper_current = data.position[0]

    def get_current_gripper_position(self):
        "get the current angle of the gripper"
        return self.__position_gripper_current

    def lock_orientation_as_is(self):
        "Lock orientation based on current orientation"
        current = self.get_current_position()
        self.lock_orientation(current.M)


    def lock_orientation(self, orientation):
        """Lock orientation, expect a PyKDL rotation matrix (PyKDL.Rotation)"""
        q = Quaternion()
        q.x, q.y, q.z, q.w = orientation.GetQuaternion()
        self.__lock_orientation_pub.publish(q);


    def unlock_orientation(self):
        "Unlock orientation"
        e = Empty()
        self.__unlock_orientation_pub.publish(e);
