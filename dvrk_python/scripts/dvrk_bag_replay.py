#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2021-06-24

# (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_bag_replay.py -a PSM1 -b /home/anton/2021-06-24-10-55-04.bag -t /PSM1/local/measured_cp

import dvrk
import sys
import time
import rospy
import rosbag
import numpy
import PyKDL
import argparse

if sys.version_info.major < 3:
    input = raw_input

# ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
rospy.init_node('dvrk_bag_replay', anonymous=True)
# strip ros arguments
argv = rospy.myargv(argv=sys.argv)

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type = str, required = True,
                    choices = ['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                    help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
parser.add_argument('-i', '--interval', type = float, default = 0.01,
                    help = 'expected interval in seconds between messages sent by the device')
parser.add_argument('-b', '--bag', type = argparse.FileType('r'), required = True,
                    help = 'ros bag containing the trajectory to replay.  The script assumes the topic to use is /<arm>/setpoint_cp.  You can change the topic used with the -t option')
parser.add_argument('-t', '--topic', type = str,
                    help = 'topic used in the ros bag.  If not set, the script will use /<arm>/setpoint_cp.  Other examples: /PSM1/local/setpoint_cp.  This is useful if you recorded the trajectory on a PSM with a base-frame defined in the console.json and you are replaying the trajectory on a PSM without a base-frame (e.g. default console provided with PSM in kinematic simulation mode.  This option allows to use the recorded setpoints without base-frame')

args = parser.parse_args(argv[1:]) # skip argv[0], script name

if args.topic is None:
    topic = '/' + args.arm + '/setpoint_cp'
else:
    topic = args.topic

# info
print('-- This script will use the topic %s\n   to replay a trajectory on arm %s' % (topic, args.arm))

# parse bag and create list of points
bbmin = numpy.zeros(3)
bbmax = numpy.zeros(3)
last_message_time = 0.0
out_of_order_counter = 0
poses = []

print('-- Parsing bag %s' % (args.bag.name))
for bag_topic, bag_message, t in rosbag.Bag(args.bag.name).read_messages():
    if bag_topic == topic:
        # check order of timestamps, drop if out of order
        transform_time = bag_message.header.stamp.to_sec()
        if transform_time <= last_message_time:
            out_of_order_counter = out_of_order_counter + 1
        else:
            # append message
            poses.append(bag_message)
            # keep track of workspace
            position = numpy.array([bag_message.transform.translation.x,
                                    bag_message.transform.translation.y,
                                    bag_message.transform.translation.z])
            if len(poses) == 1:
                bbmin = position
                bbmax = position
            else:
                bbmin = numpy.minimum(bbmin, position)
                bbmax = numpy.maximum(bbmax, position)

print('-- Found %i setpoints using topic %s' % (len(poses), topic))
if len(poses) == 0:
    print ('-- No trajectory found!')
    sys.exit()

# report out of order setpoints
if out_of_order_counter > 0:
    print('-- Found and removed %i out of order setpoints' % (out_of_order_counter))

# convert to mm
bbmin = bbmin * 1000.0
bbmax = bbmax * 1000.0
print ('-- Range of motion in mm:\n   X:[%f, %f]\n   Y:[%f, %f]\n   Z:[%f, %f]'
       % (bbmin[0], bbmax[0], bbmin[1], bbmax[1], bbmin[2], bbmax[2]))

# compute duration
duration = poses[-1].header.stamp.to_sec() - poses[0].header.stamp.to_sec()
print ('-- Duration of trajectory: %f seconds' % (duration))

# send trajectory to arm
arm = dvrk.arm(arm_name = args.arm,
               expected_interval = args.interval)

# make sure the arm is powered
print('-- Enabling arm')
if not arm.enable(10):
    sys.exit('-- Failed to enable within 10 seconds')

print('-- Homing arm')
if not arm.home(10):
    sys.exit('-- Failed to home within 10 seconds')

input('---> Make sure the arm is ready to move using cartesian positions.  For a PSM or ECM, you need to have a tool in place and the tool tip needs to be outside the cannula.  You might have to manually adjust your arm.  Press "\Enter" when the arm is ready.')

input('---> Press \"Enter\" to move to start position')

# Create frame using first pose and use blocking move
cp = PyKDL.Frame()
cp.p = PyKDL.Vector(poses[0].transform.translation.x,
                    poses[0].transform.translation.y,
                    poses[0].transform.translation.z)
cp.M = PyKDL.Rotation.Quaternion(poses[0].transform.rotation.x,
                                 poses[0].transform.rotation.y,
                                 poses[0].transform.rotation.z,
                                 poses[0].transform.rotation.w)
arm.move_cp(cp).wait()

# Replay
input('---> Press \"Enter\" to replay trajectory')

last_time = poses[0].header.stamp.to_sec()

counter = 0
total = len(poses)
start_time = time.time()

for pose in poses:
    # crude sleep to emulate period.  This doesn't take into account
    # time spend to compute cp from pose and publish so this will
    # definitely be slower than recorded trajectory
    new_time = pose.header.stamp.to_sec()
    time.sleep(new_time - last_time)
    last_time = new_time
    cp.p = PyKDL.Vector(pose.transform.translation.x,
                        pose.transform.translation.y,
                        pose.transform.translation.z)
    cp.M = PyKDL.Rotation.Quaternion(pose.transform.rotation.x,
                                     pose.transform.rotation.y,
                                     pose.transform.rotation.z,
                                     pose.transform.rotation.w)
    arm.servo_cp(cp)
    counter = counter + 1
    sys.stdout.write('\r-- Progress %02.1f%%' % (float(counter) / float(total) * 100.0))
    sys.stdout.flush()

print('\n--> Time to replay trajectory: %f seconds' % (time.time() - start_time))
print('--> Done!')
