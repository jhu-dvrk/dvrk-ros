#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import rospy
import crtk

class suj(object):
    """Simple arm API wrapping around ROS messages
    """

    # local kinematics
    class __Local:
        def __init__(self, ros_namespace, expected_interval):
            self.__crtk_utils = crtk.utils(self, ros_namespace, expected_interval)
            self.__crtk_utils.add_measured_cp()

    # initialize the arm
    def __init__(self, arm_name, ros_namespace = 'SUJ/', expected_interval = 1.0):
        """Constructor.  This initializes a few data members.It
        requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `SUJ/PSM1`"""
        # data members, event based
        self.__arm_name = arm_name
        self.__ros_namespace = ros_namespace
        self.__full_ros_namespace = self.__ros_namespace + self.__arm_name

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__full_ros_namespace, expected_interval)

        # add crtk features that we need and are supported by the dVRK
        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_measured_js()
        self.__crtk_utils.add_measured_cp()
        self.__crtk_utils.add_move_jp()

        self.local = self.__Local(self.__full_ros_namespace + '/local', expected_interval)

        # create node
        if not rospy.get_node_uri():
            rospy.init_node('arm_suj_api_' + self.__arm_name, anonymous = True, log_level = rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')
