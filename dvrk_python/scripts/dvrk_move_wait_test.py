#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2021-01-29

# (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_arm_test.py <arm-name>

import dvrk
import math
import sys
import time
import rospy
import numpy
import PyKDL
import argparse

if sys.version_info.major < 3:
    input = raw_input

# ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
rospy.init_node('dvrk_move_wait_test', anonymous = True)

# strip ros arguments
argv = rospy.myargv(argv=sys.argv)

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type=str, required=True,
                    choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                    help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
parser.add_argument('-i', '--interval', type=float, default=0.01,
                    help = 'expected interval in seconds between messages sent by the device')
args = parser.parse_args(argv[1:]) # skip argv[0], script name

arm = dvrk.arm(arm_name = args.arm,
               expected_interval = args.interval)

print('starting move_jp')

# get current position
initial_joint_position = numpy.copy(arm.setpoint_jp())
amplitude = math.radians(10.0)
goal = numpy.copy(initial_joint_position)

print('--> Testing the trajectory with wait()')

start_time = time.time()

# first motion
goal[0] = initial_joint_position[0] + amplitude
arm.move_jp(goal).wait()

# second motion
goal[0] = initial_joint_position[0] - amplitude
arm.move_jp(goal).wait()

# back to initial position
arm.move_jp(initial_joint_position).wait()
print('--> Time for the full trajectory: %f seconds' % (time.time() - start_time))


print('--> Testing the trajectory with busy loop')
start_time = time.time()

# first motion
goal[0] = initial_joint_position[0] + amplitude
counter = 0

handle = arm.move_jp(goal)
while handle.is_busy():
    counter = counter + 1
    sys.stdout.write('\r---> Loop counter: %d' % (counter))
    sys.stdout.flush()

# second motion
goal[0] = initial_joint_position[0] - amplitude
handle = arm.move_jp(goal)
while handle.is_busy():
    counter = counter + 1
    sys.stdout.write('\r---> Loop counter: %d' % (counter))
    sys.stdout.flush()

# back to initial position
handle = arm.move_jp(initial_joint_position)
while handle.is_busy():
    counter = counter + 1
    sys.stdout.write('\r---> Loop counter: %d' % (counter))
    sys.stdout.flush()

print('')
print('--> Time for the full trajectory: %f seconds' % (time.time() - start_time))

print('--> You can change the trajectory velocity in the GUI using "%s", "Direct control" and lower the "100%%" factor.  Then re-run this program.' % (args.arm))
