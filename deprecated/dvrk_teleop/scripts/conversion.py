#!/usr/bin/env python

# Zihan Chen
# 2013-07-13
# Brief: this nodes takes master position and computes desired psm position

# ros import
import roslib; roslib.load_manifest('dvrk_teleop')
import rospy
from tf.transformations import *
from geometry_msgs.msg import Pose

# python import
import numpy as np

def poseMsgToMatrix(pose):
    t = [pose.position.x, pose.position.y,
         pose.position.z]
    r = [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w]
    return np.dot(translation_matrix(t),quaternion_matrix(r))

def matrixToPoseMsg(mat):
    t = translation_from_matrix(mat)
    r = quaternion_from_matrix(mat)
#    print t
#    print r
    p = Pose()
    p.position.x = t[0]
    p.position.y = t[1]
    p.position.z = t[2]
    p.orientation.x = r[0]
    p.orientation.y = r[1]
    p.orientation.z = r[2]
    p.orientation.w = r[3]
    return p

def main():
    # id = identity_matrix()
    # mat = np.matrix('1.0 0 0 1; 0 0.7071 -0.7071 2; 0 0.7071 0.7071 3; 0 0 0 1')

    # pose = matrixToPoseMsg(mat)
    # print pose

    # remat = poseMsgToMatrix(pose)
    # print remat
    pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

