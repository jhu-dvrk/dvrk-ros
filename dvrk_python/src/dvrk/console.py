#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

from std_msgs.msg import Bool, Float64, Empty

class console(object):
    """Simple dVRK console API wrapping around ROS messages
    """

    # initialize the console
    def __init__(self, console_namespace = ''):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_console(console_namespace)

    def __init_console(self, ral, console_name):
        """Default is console name is 'console' and it would be necessary to change it only if
        you have multiple dVRK consoles"""
        # data members, event based
        self._ral = ral.create_child(console_name)
        self._teleop_scale = 0.0

        # publishers
        self._power_off_pub = self._ral.publisher('/power_off',
                                                  Empty,
                                                  latch = True, queue_size = 1)
        self._power_on_pub = self._ral.publisher('/power_on',
                                                 Empty,
                                                 latch = True, queue_size = 1)
        self._home_pub = self._ral.publisher('/home',
                                             Empty,
                                             latch = True, queue_size = 1)
        self._teleop_enable_pub = self._ral.publisher('/teleop/enable',
                                                      Bool,
                                                      latch = True, queue_size = 1)
        self._teleop_set_scale_pub = self._ral.publisher('/teleop/set_scale',
                                                         Float64,
                                                         latch = True, queue_size = 1)

        # subscribers
        self._teleop_scale_sub = self._ral.subscriber('/teleop/scale',
                                                      Float64,
                                                      self.__teleop_scale_cb)

    def __teleop_scale_cb(self, data):
        """Callback for teleop scale.

        :param data: the latest scale requested for the dVRK console"""
        self.__teleop_scale = data.data

    def power_off(self):
        self.__power_off_pub.publish()

    def power_on(self):
        self.__power_on_pub.publish()

    def home(self):
        self.__home_pub.publish()

    def teleop_start(self):
        self.__teleop_enable_pub.publish(True)

    def teleop_stop(self):
        self.__teleop_enable_pub.publish(False)

    def teleop_set_scale(self, scale):
        self.__teleop_set_scale_pub.publish(scale)

    def teleop_get_scale(self):
        return self.__teleop_scale
