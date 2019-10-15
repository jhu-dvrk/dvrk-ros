#!/usr/bin/env python

# Authors: Anton Deguet
# Date: 2018-09-10
# Copyright JHU 2010

# Todo:
# - test calibrating 3rd offset on PSM?

import time
import rospy
import math
import sys
import csv
import datetime
import dvrk
import numpy

def dvrk_ecm_gc_collect(robot_name):
    # create dVRK robot
    robot = dvrk.ecm(robot_name)

    # file to save data
    now = datetime.datetime.now()
    now_string = now.strftime("%Y-%m-%d-%H-%M")
    csv_file_name = 'ecm-gc-' + now_string + '.csv'
    print("Values will be saved in: ", csv_file_name)
    f = open(csv_file_name, 'wb')
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

        robot.move_joint(numpy.array([positions[0],
                                      positions[1],
                                      positions[2],
                                      0.0]))
        time.sleep(1.0)
        writer.writerow([robot.get_current_joint_position()[0],
                         robot.get_current_joint_position()[1],
                         robot.get_current_joint_position()[2],
                         robot.get_current_joint_effort()[0],
                         robot.get_current_joint_effort()[1],
                         robot.get_current_joint_effort()[2]])

    f.close()
    robot.move_joint(numpy.array([0.0, 0.0, 0.0, 0.0]))

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires arm name, e.g. ECM'
    else:
        dvrk_ecm_gc_collect(sys.argv[1])
