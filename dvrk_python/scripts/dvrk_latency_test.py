#!/usr/bin/env python

# Author: Adnan Munawar
# Testing Robot IO Loading with varying ROS Communication Load

from __future__ import print_function
from dvrk import arm, psm, mtm, ecm
import rospy
import time
from threading import Thread
from cisst_msgs.msg import mtsIntervalStatistics as StatsMsg
import sys
import rosbag
import datetime
import os


class Stats(object):
    def __init__(self):
        rospy.init_node('dvrk_roscomm_load_test')
        self._rate = rospy.Rate(100)
        self._userDataScale = 1
        self._userData = 0
        self._active = True
        self._dir_name = 'bag'
        if not os.path.exists(self._dir_name):
            os.mkdir(self._dir_name)
        self._bag_name = self._dir_name + '/latency_test:' + datetime.datetime.now().strftime("(%Y-%m-%d)(%H:%M:%S)") + '.bag'
        self._bag = rosbag.Bag(self._bag_name, 'w')

        self._pub_stat_msg = StatsMsg()
        self._io_stat_msg  = StatsMsg()
        self._tf_stat_msg  = StatsMsg()
        self._spin_stat_msg = StatsMsg()

        _name_space       = '/dvrk/'
        _cisst_msg_suffix = '/period_statistics'
        _user_msg_suffix  = '/eval'
        _msg_name_dict    = {'pub' : 'pubBridge',
                             'io'  : 'io'       ,
                             'tf'  : 'tfBridge' ,
                             'spin': 'spinBridge'}

        self._pubBridgeStatsTopicPubStr  = _name_space + _msg_name_dict['pub']  + _user_msg_suffix
        self._ioBridgeStatsTopicPubStr   = _name_space + _msg_name_dict['io']   + _user_msg_suffix
        self._tfBridgeStatsTopicPubStr   = _name_space + _msg_name_dict['tf']   + _user_msg_suffix
        self._spinBridgeStatsTopicPubStr = _name_space + _msg_name_dict['spin'] + _user_msg_suffix

        self._pubBridgeStatsTopicSubStr  = _name_space + _msg_name_dict['pub']  + _cisst_msg_suffix
        self._ioBridgeStatsTopicSubStr   = _name_space + _msg_name_dict['io']   + _cisst_msg_suffix
        self._tfBridgeStatsTopicSubStr   = _name_space + _msg_name_dict['tf']   + _cisst_msg_suffix
        self._spinBridgeStatsTopicSubStr = _name_space + _msg_name_dict['spin'] + _cisst_msg_suffix

        self._pubBpub  = rospy.Publisher(self._pubBridgeStatsTopicPubStr, StatsMsg,  queue_size=10)
        self._ioBpub   = rospy.Publisher(self._ioBridgeStatsTopicPubStr, StatsMsg,   queue_size=10)
        self._tfBpub   = rospy.Publisher(self._tfBridgeStatsTopicPubStr, StatsMsg,   queue_size=10)
        self._spinBpub = rospy.Publisher(self._spinBridgeStatsTopicPubStr, StatsMsg, queue_size=10)

        self._pub_dict = {self._pub_stat_msg  : self._pubBpub,
                          self._io_stat_msg   : self._ioBpub ,
                          self._tf_stat_msg   : self._tfBpub ,
                          self._spin_stat_msg : self._spinBpub
                          }

        self._pubBsub  = rospy.Subscriber(self._pubBridgeStatsTopicSubStr, StatsMsg,
                                        self._pubB_cb, queue_size=10, tcp_nodelay=True)
        self._ioBsub   = rospy.Subscriber(self._ioBridgeStatsTopicSubStr, StatsMsg,
                                        self._ioB_cb, queue_size=10, tcp_nodelay=True)
        self._tfBsub   = rospy.Subscriber(self._tfBridgeStatsTopicSubStr, StatsMsg,
                                        self._tfB_cb, queue_size=10, tcp_nodelay=True)
        self._spinBsub = rospy.Subscriber(self._spinBridgeStatsTopicSubStr, StatsMsg,
                                        self._spinB_cb, queue_size=10, tcp_nodelay=True)

        self._sub_list = [self._pubBsub, self._ioBsub, self._tfBsub, self._spinBsub]

        self._pubThread = Thread(target=self._run_pubs)
        self._pubThread.daemon = True
        self._pubThread.start()

    def set_user_data(self, n_arms):
        self._userData = n_arms * self._userDataScale
        pass

    def clear_user_data(self):
        self._userData = 0
        pass

    def disconnect(self):
        self._active = False
        self._bag.close()

    def _pubB_cb(self, data):
        if self._active:
            self._pub_stat_msg = data
            self._pub_stat_msg.UserData = self._userData

    def _ioB_cb(self, data):
        if self._active:
            self._io_stat_msg = data
            self._io_stat_msg.UserData = self._userData

    def _tfB_cb(self, data):
        if self._active:
            self._tf_stat_msg = data
            self._tf_stat_msg.UserData = self._userData

    def _spinB_cb(self, data):
        if self._active:
            self._spin_stat_msg = data
            self._spin_stat_msg.UserData = self._userData

    def _run_pubs(self):
        while not rospy.is_shutdown() and self._active:
            for msg, pub in self._pub_dict.iteritems():
                pub.publish(msg)
            self._bag.write(self._pubBridgeStatsTopicPubStr, self._pub_stat_msg)
            self._bag.write(self._ioBridgeStatsTopicPubStr, self._io_stat_msg)
            self._bag.write(self._tfBridgeStatsTopicPubStr, self._tf_stat_msg)
            self._bag.write(self._spinBridgeStatsTopicPubStr, self._spin_stat_msg)

            self._rate.sleep()


class DvrkLatencyTest(Stats):
    def __init__(self):
        super(DvrkLatencyTest, self).__init__()
        self.psmInterface = psm
        self.mtmInterface = mtm
        self.ecmInterface = ecm
        self.arm_dict = {'PSM1': self.psmInterface,
                         'PSM2': self.psmInterface,
                         'PSM3': self.psmInterface,
                         'MTMR': self.mtmInterface,
                         'MTML': self.mtmInterface,
                         'ECM' : self.ecmInterface}
        self.activeArms = []
        self.maxArms = 6

    def create_arm_load(self, n_arms, delay = 0.0):
        self._is_narm_valid(n_arms, self.arm_dict.__len__(), 1)
        cnt = 0
        for armStr, arm_irfc in self.arm_dict.iteritems():
            arm_irfc = arm_irfc(armStr)
            self.activeArms.append(arm_irfc)
            cnt += 1
            self.set_user_data(self.activeArms.__len__())
            print('Connecting ROS Client for {}'.format(arm_irfc.name()))
            time.sleep(delay)
            if cnt == n_arms:
                break

    def relieve_arm_load(self, n_arms=None, delay = 0.0):
        n_active_arms = self.activeArms.__len__()

        if n_arms is None:
            n_arms = n_active_arms

        self._is_narm_valid(n_arms, n_active_arms)
        for i in range(n_arms):
            arm_irfc = self.activeArms.pop()
            arm_irfc.unregister()
            self.set_user_data(self.activeArms.__len__())
            print('Disconnecting ROS Client for {}'.format(arm_irfc.name()))
            time.sleep(delay)

    def _is_narm_valid(self, n_arms, max_num=6, min_num=0):
        if n_arms < min_num or n_arms > max_num:
            raise ValueError('num_arms cannot be negative or greater than {}'.format(max_num))


def test_load(dt = 0.5):
    lat_test = DvrkLatencyTest()
    for i in range(1, lat_test.maxArms + 1):
        n_arms = i % (lat_test.maxArms+1)
        lat_test.create_arm_load(n_arms, delay=dt)
        time.sleep(dt*1)
        lat_test.relieve_arm_load(delay=dt)
        time.sleep(dt*1)
    lat_test.disconnect()


if __name__ == '__main__':
    args = sys.argv
    dt = 0.5
    if args.__len__() > 1:
        dt_str = args[1]
        try:
            float(dt_str)
        except ValueError:
            raise ValueError('Expecting a number, got string {}'.format(dt_str))
        dt = float(dt_str)
        print(dt)
        if not 0 <= dt <= 10:
            raise ValueError('Delay between loading arms should be between {} : {}'.format(0.0, 10.0))
    test_load(dt)
