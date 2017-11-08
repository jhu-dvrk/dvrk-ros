#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2017 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import rospy

from std_msgs.msg import Bool, Float32, Empty

class console(object):
    """Simple dVRK console API wrapping around ROS messages
    """

    # initialize the console
    def __init__(self, console_namespace = '/dvrk/console'):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_console(console_namespace)


    def __init_console(self, console_namespace = '/dvrk/console'):
        """Constructor.  This initializes a few data members. It
        requires a arm name, this will be used to find the ROS topics
        for the console being controlled.  The default is
        '/dvrk/console' and it would be necessary to change it only if
        you have multiple dVRK consoles"""
        # data members, event based
        self.__console_namespace = console_namespace
        self.__teleop_scale = 0.0

        # publishers
        self.__power_off_pub = rospy.Publisher(self.__console_namespace
                                               + '/power_off',
                                               Empty, latch = True, queue_size = 1)
        self.__power_on_pub = rospy.Publisher(self.__console_namespace
                                              + '/power_on',
                                              Empty, latch = True, queue_size = 1)
        self.__home_pub = rospy.Publisher(self.__console_namespace
                                          + '/home',
                                          Empty, latch = True, queue_size = 1)
        self.__teleop_enable_pub = rospy.Publisher(self.__console_namespace
                                                   + '/teleop/enable',
                                                   Bool, latch = True, queue_size = 1)
        self.__teleop_set_scale_pub = rospy.Publisher(self.__console_namespace
                                                      + '/teleop/set_scale',
                                                      Float32, latch = True, queue_size = 1)

        # subscribers
        rospy.Subscriber(self.__console_namespace
                         + '/teleop/scale',
                         Float32, self.__teleop_scale_cb)

        # create node
        if not rospy.get_node_uri():
            rospy.init_node('console_api', anonymous = True, log_level = rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')


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
