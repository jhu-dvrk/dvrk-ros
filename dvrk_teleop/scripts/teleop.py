#!/usr/bin/env python

# Zihan Chen
# 2013-07-13
# Brief: this nodes takes master position and computes desired psm position

# ros import
import roslib; roslib.load_manifest('dvrk_teleop')
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

# python import
import numpy as np
from conversion import *

class Teleoperation():

    def __init__(self, name):
        # ros init
        rospy.init_node(name)

        # ros subscriber
        sub_master_pose = rospy.Subscriber('/dvrk_mtm/cartesian_pose_current',
                                           Pose,
                                           self.master_pose_cb)

        sub_slave_pose = rospy.Subscriber('/dvrk_psm/cartesian_pose_current',
                                          Pose,
                                          self.slave_pose_cb)

        sub_footpedal_clutch = rospy.Subscriber('/dvrk_footpedal/clutch_state',
                                                Bool,
                                                self.footpedal_clutch_cb)

        # ros publisher
        self.pub_pose_ = rospy.Publisher('/dvrk_psm/cartesian_pose_command', Pose)
        self.rate_ = rospy.Rate(50)  # 50 hz

        # variable
        self.counter_ = 0
        self.mtm_mat_cur = identity_matrix()
        self.psm_mat_cur = identity_matrix()
        self.mtm_mat_prev = identity_matrix()
        self.psm_mat_prev = identity_matrix()
        self.is_master_cb_initialized = False
        self.is_slave_cb_initialized = False

        self.deltaX = 0.0005
        self.is_clutch_pressed_ = False;

    def master_pose_cb(self, data):
        self.mtm_mat_prev = self.mtm_mat_cur
        self.mtm_mat_cur = poseMsgToMatrix(data)
        self.is_master_cb_initialized = True
#        print self.mtm_mat_cur
        pass

    def slave_pose_cb(self, data):
        self.psm_mat_prev = self.psm_mat_cur
        self.psm_mat_cur = poseMsgToMatrix(data)
        self.is_slave_cb_initialized = True
#        print self.psm_mat_cur
        pass

    def footpedal_clutch_cb(self, data):
        if (data.data == True):
            self.is_clutch_pressed_ = False
        else:
            self.is_clutch_pressed_ = True
#        print self.is_clutch_pressed_
        pass
    
    def run(self):
        while not rospy.is_shutdown():
            if (not self.is_clutch_pressed_):
                # do computation here
                # msg_pose = self.psm_mat_cur
                # if (msg_pose.position.x > 0.07):
                #     self.deltaX = -0.0005
                # elif (msg_pose.position.x < -0.07):
                #     self.deltaX = 0.0005
                # msg_pose.position.x += self.deltaX

                # get master pose
                scale = 1.0
                mtm_tra_cur = translation_from_matrix(self.mtm_mat_cur)
                mtm_tra_pre = translation_from_matrix(self.mtm_mat_prev)
                psm_tra_cmd = scale * (mtm_tra_cur -mtm_tra_pre)

                mtm2psm = np.array([[-1.0, 0.0, 0.0],
                                    [0.0, -1.0, 0.0],
                                    [0.0, 0.0,  1.0]])
                psm_tra_cmd = np.dot(mtm2psm, psm_tra_cmd)

                if (self.counter_%5 == 0):
                    print psm_tra_cmd
                
                # publish
                msg_pose = matrixToPoseMsg(self.psm_mat_cur)
                self.pub_pose_.publish(msg_pose)

                # save to prev
#                self.mtm_mat_prev = self.mtm_mat_cur
#                self.psm_mat_prev = self.psm_mat_cur
            else:
                # do nothing now
                if (self.counter_%10 == 0):
                    rospy.loginfo("MTM or PSM not ready")
                pass

            self.counter_ += 1
            self.rate_.sleep()
        pass

if __name__ == '__main__':
    try:
        print "... Starting dvrk_teleop ..."
        teleop = Teleoperation("teleop_zihan")
        teleop.run()
    except rospy.ROSInterruptException:
        pass

