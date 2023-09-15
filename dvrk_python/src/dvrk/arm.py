#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.

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

import crtk
import std_msgs.msg

class arm(object):
    """Simple arm API wrapping around ROS messages"""

    # class to contain spatial/body cf methods
    class __MeasuredServoCf:
        def __init__(self, ral, expected_interval):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval)
            self.__crtk_utils.add_measured_cf()
            self.__crtk_utils.add_servo_cf()
            self.__crtk_utils.add_jacobian()

    # local kinematics
    class __Local:
        def __init__(self, ral, expected_interval):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval)
            self.__crtk_utils.add_measured_cp()
            self.__crtk_utils.add_setpoint_cp()
            self.__crtk_utils.add_forward_kinematics()

    # initialize the arm
    def __init__(self, ral, arm_name, expected_interval = 0.01):
        """Requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `PSM1`"""
        self.__name = arm_name
        self.__ral = ral.create_child(arm_name)

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__ral, expected_interval)

        # add crtk features that we need and are supported by the dVRK
        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_setpoint_js()
        self.__crtk_utils.add_setpoint_cp()
        self.__crtk_utils.add_measured_js()
        self.__crtk_utils.add_measured_cp()
        self.__crtk_utils.add_measured_cv()
        self.__crtk_utils.add_hold()
        self.__crtk_utils.add_free()
        self.__crtk_utils.add_servo_jp()
        self.__crtk_utils.add_servo_jr()
        self.__crtk_utils.add_servo_cp()
        self.__crtk_utils.add_servo_jf()
        self.__crtk_utils.add_move_jp()
        self.__crtk_utils.add_move_jr()
        self.__crtk_utils.add_move_cp()
        self.__crtk_utils.add_forward_kinematics()
        self.__crtk_utils.add_inverse_kinematics()

        self.spatial = self.__MeasuredServoCf(self.__ral.create_child('spatial'), expected_interval)
        self.body = self.__MeasuredServoCf(self.__ral.create_child('body'), expected_interval)
        self.local = self.__Local(self.__ral.create_child('local'), expected_interval)

        # publishers
        self.__body_set_cf_orientation_absolute_pub = self.__ral.publisher(
            '/body/set_cf_orientation_absolute', std_msgs.msg.Bool,
            latch = True, queue_size = 1)
        self.__use_gravity_compensation_pub = self.__ral.publisher(
            '/use_gravity_compensation', std_msgs.msg.Bool,
            latch = True, queue_size = 1)
        self.__trajectory_j_set_ratio_pub = self.__ral.publisher(
            '/trajectory_j/set_ratio', std_msgs.msg.Float64,
            latch = True, queue_size = 1)

        self.__trajectory_j_ratio = 1.0
        self.__trajectory_j_ratio_sub = self.__ral.subscriber(
            '/trajectory_j/ratio', std_msgs.msg.Float64,
            self.__trajectory_j_ratio_cb)

    def name(self):
        return self.__name

    def ral(self):
        return self.__ral
    
    def check_connections(self, timeout = 5.0):
        self.__ral.check_connections(timeout)

    def body_set_cf_orientation_absolute(self, absolute):
        """Apply body wrench using body orientation (relative/False) or reference frame (absolute/True)"""
        m = std_msgs.msg.Bool()
        m.data = absolute
        self.__body_set_cf_orientation_absolute_pub.publish(m)

    def use_gravity_compensation(self, gravity_compensation):
        """Turn on/off gravity compensation in cartesian effort mode"""
        g = std_msgs.msg.Bool()
        g.data = gravity_compensation
        self.__use_gravity_compensation_pub.publish(g)

    def trajectory_j_set_ratio(self, ratio):
        """Set ratio applied to max velocities and accelerations used for
        joint trajectory generation.  Value should be in range ]0, 1]
        """
        r = std_msgs.msg.Float64()
        r.data = ratio
        self.__trajectory_j_set_ratio_pub.publish(r)

    def trajectory_j_get_ratio(self):
        return self.__trajectory_j_ratio

    def __trajectory_j_ratio_cb(self, msg):
        self.__trajectory_j_ratio = msg.data

    def trajectory_j_ratio(self):
        return self.__trajectory_j_ratio
