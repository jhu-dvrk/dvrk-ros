#!/usr/bin/env python

# Zihan Chen
# 2013-07-13
# Brief: this nodes takes master position and computes desired psm position

# ros import
import roslib; roslib.load_manifest('irk_teleop')
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
# from sawROS.msg import
# from sensor_msgs.msg import

# python import
import numpy as np

class Teleoperation():

    def __init__(self, name):
        # ros init
        rospy.init_node(name)

        # ros subscriber
        sub_master_pose = rospy.Subscriber('/irk_mtm/cartesian_pose_current',
                                           Pose,
                                           self.master_pose_cb)

        sub_slave_pose = rospy.Subscriber('/irk_psm/cartesian_pose_current',
                                          Pose,
                                          self.slave_pose_cb)

        # ros publisher
        self.pub_pose_ = rospy.Publisher('/irk_psm/cartesian_pose_command', Pose)
        self.rate_ = rospy.Rate(50)  # 50 hz

    def master_pose_cb(self, data):
        print data
        pass

    def slave_pose_cb(self, data):
        print data
        pass
    
    def run(self):
        while not rospy.is_shutdown():
            # do computation here
            msg_pose = Pose()
            self.pub_pose_.publish(msg_pose)
            self.rate_.sleep()
        pass

if __name__ == '__main__':
    try:
        print "... Starting irk_teleop ..."
        teleop = Teleoperation("teleop_zihan")
        teleop.run()
    except rospy.ROSInterruptException:
        pass

