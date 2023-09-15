#  Author(s):  Anton Deguet
#  Created on: 2016-05

#   (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

from dvrk.arm import *

import math
import numpy

class psm(arm):
    """Simple robot API wrapping around ROS messages
    """

    # class to contain jaw methods
    class __Jaw:
        def __init__(self, ral, expected_interval, operating_state_instance):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval, operating_state_instance)
            self.__crtk_utils.add_measured_js()
            self.__crtk_utils.add_setpoint_js()
            self.__crtk_utils.add_servo_jp()
            self.__crtk_utils.add_move_jp()
            self.__crtk_utils.add_servo_jf()

        def close(self):
            "Close the tool jaw"
            return self.move_jp(numpy.array([math.radians(-20.0)]))

        def open(self, angle = math.radians(60.0)):
            "Close the tool jaw"
            return self.move_jp(numpy.array([angle]))


    # initialize the robot
    def __init__(self, ral, arm_name, expected_interval = 0.01):
        # first call base class constructor
        super().__init__(ral, arm_name, expected_interval)
        jaw_ral = self.ral().create_child('/jaw')
        self.jaw = self.__Jaw(jaw_ral, expected_interval,
                              operating_state_instance = self)

        # publishers
        self.__set_tool_present_publisher = self.ral().publisher('/emulate_tool_present',
                                                                 std_msgs.msg.Bool,
                                                                 latch = True, queue_size = 1)

    def insert_jp(self, depth):
        "insert the tool, by moving it to an absolute depth"
        goal = numpy.copy(self.setpoint_jp())
        goal[2] = depth
        return self.move_jp(goal)

    def set_tool_present(self, tool_present):
        "Set tool inserted.  To be used only for custom tools that can't be detected automatically"
        tp = std_msgs.msg.Bool()
        tp.data = tool_present
        self.__set_tool_present_publisher.publish(tp)
