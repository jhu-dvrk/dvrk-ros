#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2021-06-24

# (C) Copyright 2021-2023 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import crtk
import rospy
import numpy
import PyKDL
import argparse
import sys
import math
import time

# simplified arm class for the PSM
class simple_psm:

    # simplified jaw class to control the jaws
    class __jaw_device:
        def __init__(self, ral,
                     expected_interval, operating_state_instance):
            self.__crtk_utils = crtk.utils(self, ral,
                                           expected_interval, operating_state_instance)
            self.__crtk_utils.add_servo_jf() # to control jaw effort

    def __init__(self, ral, expected_interval):
        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ral, expected_interval)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_servo_jf()    # to make arm limp
        self.crtk_utils.add_measured_cp() # to measure actual cartesian pose
        self.jaw = self.__jaw_device(ral.create_child('jaw'),
                                     expected_interval,
                                     operating_state_instance = self)

# simplified arm class for the ECM
class simple_ecm:

    def __init__(self, ral, expected_interval):
        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ral, expected_interval)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_move_jp()
        self.crtk_utils.add_setpoint_js()
        self.crtk_utils.add_move_cp()
        self.crtk_utils.add_setpoint_cp()

if sys.version_info.major < 3:
    input = raw_input

# extract ros arguments (e.g. __ns:= for namespace)
argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type = str, required = True,
                    choices = ['PSM1', 'PSM2', 'PSM3'],
                    help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
parser.add_argument('-i', '--interval', type = float, default = 0.01,
                    help = 'expected interval in seconds between messages sent by the device')

args = parser.parse_args(argv)

ral = crtk.ral('dvrk_psm_ecm_registration')

# create PSM and ECM
psm = simple_psm(ral = ral.create_child(args.arm),
                 expected_interval = args.interval)

ecm = simple_ecm(ral = ral.create_child('ECM'),
                 expected_interval = args.interval)
ral.check_connections()
ral.spin()

input('-> Press "Enter" to align ECM')
ecm_setpoint_jp = ecm.setpoint_jp()
ecm_setpoint_jp[0] = 0.0
ecm_setpoint_jp[1] = 0.0
ecm_setpoint_jp[3] = 0.0
ecm.move_jp(ecm_setpoint_jp).wait()

input('-> Press "Enter" to close the PSM jaw')
psm.jaw.servo_jf(numpy.array([-0.1]))

input('-> Press "Enter" to start ECM motion')

# ECM base to tip
ecm_t_b = PyKDL.Frame()
ecm_t_b.p = ecm.setpoint_cp().p
ecm_t_b.M = ecm.setpoint_cp().M

goal = PyKDL.Frame()
goal.p = ecm.setpoint_cp().p
goal.M = ecm.setpoint_cp().M

amplitude = 0.02 # 1/2 of the full motion

# X motion
disp = PyKDL.Vector(0.0, 0.0, 0.0)
disp[0] = -amplitude
goal.p = ecm_t_b.p + ecm_t_b.M * disp
ecm.move_cp(goal).wait()
time.sleep(0.5)
psm_x0 = psm.measured_cp().p

disp[0] = amplitude
goal.p = ecm_t_b.p + ecm_t_b.M * disp
ecm.move_cp(goal).wait()
time.sleep(0.5)
psm_x1 = psm.measured_cp().p

psm_x = psm_x1 - psm_x0
print('-- Amplitude of x motion: %f' % (psm_x.Normalize()))

# Y motion
disp[0] = 0.0
disp[1] = -amplitude
goal.p = ecm_t_b.p + ecm_t_b.M * disp
ecm.move_cp(goal).wait()
time.sleep(0.5)
psm_y0 = psm.measured_cp().p

disp[1] = amplitude
goal.p = ecm_t_b.p + ecm_t_b.M * disp
ecm.move_cp(goal).wait()
time.sleep(0.5)
psm_y1 = psm.measured_cp().p

psm_y = psm_y1 - psm_y0
print('-- Amplitude of y motion: %f' % (psm_y.Normalize()))

# Z motion
disp[1] = 0.0
disp[2] = -amplitude
goal.p = ecm_t_b.p + ecm_t_b.M * disp
ecm.move_cp(goal).wait()
time.sleep(0.5)
psm_z0 = psm.measured_cp().p

disp[2] = amplitude
goal.p = ecm_t_b.p + ecm_t_b.M * disp
ecm.move_cp(goal).wait()
time.sleep(0.5)
psm_z1 = psm.measured_cp().p

psm_z = psm_z1 - psm_z0
print('-- Amplitude of z motion: %f' % (psm_z.Normalize()))

# back to zero
ecm.move_cp(ecm_t_b).wait()

# quick sanity checks, should be close-ish to 90 degrees
print ('-- Angle between x and y: %f' % (math.degrees(math.acos(PyKDL.dot(psm_x, psm_y)))))
print ('-- Angle between y and z: %f' % (math.degrees(math.acos(PyKDL.dot(psm_y, psm_z)))))
print ('-- Angle between z and x: %f' % (math.degrees(math.acos(PyKDL.dot(psm_z, psm_x)))))

# convert to quaternion
q = PyKDL.Rotation(psm_x, psm_y, psm_z).GetQuaternion()

# normalize
q = q / (numpy.linalg.norm(q))

# get normalized rotational matrix
r = PyKDL.Rotation.Quaternion(q[0], q[1], q[2], q[3])

# print text to copy/paste in json file
print(',\n"base-frame": {\n"reference-frame": "ECM",\n"transform": [[ %.10f, %.10f, %.10f, 0.0],\n[ %.10f, %.10f, %.10f, 0.0],\n[%.10f, %.10f, %.10f, 0.0],\n[0.0, 0.0, 0.0, 1.0]]\n}'
      % ( r[0,0], r[0,1], r[0,2],
          r[1,0], r[1,1], r[1,2],
          r[2,0], r[2,1], r[2,2]))

input('-> Press "Enter" to release the PSM jaw')
psm.jaw.servo_jf(numpy.array([0.0]))

ral.shutdown()
