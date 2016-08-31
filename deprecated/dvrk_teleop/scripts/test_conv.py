#!/usr/bin/env python

# Zihan Chen
# 2013-07-13
# Brief: this nodes takes master position and computes desired psm position

# ros import
import roslib; roslib.load_manifest('dvrk_teleop')
import rospy
from conversion import *

# python import
import numpy as np

def main():
    id = identity_matrix()
#    print id
    mat = np.matrix('1.0 0 0 1; 0 0.7071 -0.7071 2; 0 0.7071 0.7071 3; 0 0 0 1')

    pose = matrixToPoseMsg(mat)
    print pose

    remat = poseMsgToMatrix(pose)
    print remat


if __name__ == '__main__':
    main()


