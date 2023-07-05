#  Author(s):  Anton Deguet
#  Created on: 2016-08

# (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

from std_msgs.msg import Float64, String

class teleop_psm(object):
    """Simple dVRK teleop PSM API wrapping around ROS messages"""

    def __init__(self, ral, teleop_name):
        """Requires a teleop name, this will be used to find the ROS topics
        for the console being controlled."""
        # data members
        self._ral = ral.create_child(teleop_name)
        self._scale = 0.0

        # publishers
        self._set_scale_pub = self._ral.publisher('/set_scale',
                                                  Float64,
                                                  latch = True, queue_size = 1)
        self._set_desired_state_pub = self._ral.publisher('/set_desired_state',
                                                          String,
                                                          latch = True, queue_size = 1)

        # subscribers
        self._scale_sub = self._ral.subscriber('/scale', Float64, self._scale_cb)

    def _scale_cb(self, data):
        """Callback for teleop scale.

        :param data: the latest scale requested for the teleop"""
        self._scale = data.data

    def set_scale(self, scale):
        self._set_scale_pub.publish(scale)

    def get_scale(self):
        return self._scale

    def enable(self):
        self._set_desired_state_pub.publish('ENABLED')

    def disable(self):
        self._set_desired_state_pub.publish('DISABLED')
