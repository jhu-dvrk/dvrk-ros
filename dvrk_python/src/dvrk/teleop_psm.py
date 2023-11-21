#  Author(s):  Anton Deguet
#  Created on: 2016-08

# (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import std_msgs.msg
import crtk_msgs.msg

class teleop_psm(object):
    """Simple dVRK teleop PSM API wrapping around ROS messages"""

    def __init__(self, ral, teleop_name):
        """Requires a teleop name, this will be used to find the ROS topics
        for the console being controlled."""
        # data members
        self.__ral = ral.create_child(teleop_name)
        self.__scale = 0.0

        # publishers
        self.__set_scale_pub = self.__ral.publisher('/set_scale',
                                                    std_msgs.msg.Float64,
                                                    latch = False, queue_size = 1)
        self.__set_state_command_pub = self.__ral.publisher('/state_command',
                                                            crtk_msgs.msg.StringStamped,
                                                            latch = False, queue_size = 1)

        # subscribers
        self.__scale_sub = self.__ral.subscriber('/scale', std_msgs.msg.Float64, self.__scale_cb)

    def __scale_cb(self, data):
        """Callback for teleop scale.

        :param data: the latest scale requested for the teleop"""
        self.__scale = data.data

    def set_scale(self, scale):
        msg = std_msgs.msg.Float64()
        msg.data = scale
        self.__set_scale_pub.publish(msg)

    def get_scale(self):
        return self.__scale

    def enable(self):
        msg = crtk_msgs.msg.StringStamped()
        msg.string = 'enable'
        self.__set_state_command_pub.publish(msg)

    def disable(self):
        msg = crtk_msgs.msg.StringStamped()
        msg.string = 'disable'
        self.__set_state_command_pub.publish(msg)
