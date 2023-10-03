#!/usr/bin/env python

# Authors: Anton Deguet
# Date: 2018-09-10
# Copyright JHU 2018-2023

# Simple script to collect data on ECM Classic for gravity
# compensation identification.

# This scripts needs to be use different joints limits for different
# arms, i.e. Classic vs Si, PSM vs ECM.

import time
import math
import sys
import csv
import datetime
import crtk
import dvrk
import numpy
import argparse

def dvrk_ecm_psm_gc_collect(ral, name, expected_interval):
    # create dVRK robot
    robot = dvrk.ecm(ral = ral,
                     arm_name = name,
                     expected_interval = expected_interval)
    ral.check_connections()

    # file to save data
    now = datetime.datetime.now()
    now_string = now.strftime("%Y-%m-%d-%H-%M")
    csv_file_name = name + '-gc-' + now_string + '.csv'
    print("Values will be saved in: ", csv_file_name)
    f = open(csv_file_name, 'w')
    writer = csv.writer(f)

    # compute joint limits
    d2r = math.pi / 180.0 # degrees to radians
    lower_limits = [-80.0 * d2r, -40.0 * d2r,  0.005]
    upper_limits = [ 80.0 * d2r,  60.0 * d2r,  0.230]

    # set sampling for data
    # increments = [40.0 * d2r, 40.0 * d2r, 0.10] # less samples
    increments = [20.0 * d2r, 20.0 * d2r, 0.05] # more samples
    directions = [1.0, 1.0, 1.0]

    # start position
    positions = [lower_limits[0],
                 lower_limits[1],
                 lower_limits[2]]

    all_reached = False

    robot.home();

    while not all_reached:
        next_dimension_increment = True
        for index in range(3):
            if next_dimension_increment:
                future = positions[index] + directions[index] * increments[index]
                if (future > upper_limits[index]):
                    directions[index] = -1.0
                    if index == 2:
                        all_reached = True
                elif (future < lower_limits[index]):
                    directions[index] = 1.0
                else:
                    positions[index] = future
                    next_dimension_increment = False

        robot.move_jp(numpy.array([positions[0],
                                   positions[1],
                                   positions[2],
                                   0.0])).wait()
        time.sleep(1.0)
        writer.writerow([robot.measured_jp()[0],
                         robot.measured_jp()[1],
                         robot.measured_jp()[2],
                         robot.measured_jf()[0],
                         robot.measured_jf()[1],
                         robot.measured_jf()[2]])

    f.close()
    robot.move_jp(numpy.array([0.0, 0.0, 0.0, 0.0])).wait()

if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['ECM', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_ecm_psm_gc_collect')
    ral.spin_and_execute(dvrk_ecm_psm_gc_collect, ral, args.arm, args.interval)
