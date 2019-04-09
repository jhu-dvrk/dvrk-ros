#!/usr/bin/env python

# Authors: Nick Eusman, Anton Deguet
# Date: 2015-09-24
# Copyright JHU 2015-2017

# Todo:
# - test calibrating 3rd offset on PSM?

import time
import rospy
import math
import sys
import csv
import datetime
import numpy

from sensor_msgs.msg import JointState
import dvrk
import xml.etree.ElementTree as ET


if sys.version_info.major < 3:
    input = raw_input


def slope(x, y):
    a = []
    for i in range(len(x)):
        a.append(x[i] * y[i])
    sum_a = sum(a)
    final_a = (sum_a * len(x))
    final_b = (sum(x) * sum(y))

    c_squared = []
    for i in x:
        c_squared.append(i**2)

    c_sum = sum(c_squared)
    final_c = (c_sum * len(x))
    final_d = (sum(x)**2)

    result = (final_a - final_b) / (final_c - final_d)
    return result

class potentiometer_calibration:

    def __init__(self, robot_name):
        self._robot_name = robot_name
        self._serial_number = ""
        self._data_received = False # use pots to make sure the ROS topics are OK
        self._last_potentiometers = []
        self._last_joints = []
        ros_namespace = '/dvrk/' + self._robot_name
        rospy.Subscriber(ros_namespace +  '/io/analog_input_pos_si', JointState, self.pot_callback)
        rospy.Subscriber(ros_namespace +  '/io/joint_position', JointState, self.joints_callback)

    def pot_callback(self, data):
        self._last_potentiometers[:] = data.position
        self._data_received = True

    def joints_callback(self, data):
        self._data_received = True
        self._last_joints[:] = data.position


    def run(self, calibrate, filename):
        nb_joint_positions = 20 # number of positions between limits
        nb_samples_per_position = 500 # number of values collected at each position
        total_samples = nb_joint_positions * nb_samples_per_position
        samples_so_far = 0

        sleep_time_after_motion = 0.5 # time after motion from position to position to allow potentiometers to stabilize
        sleep_time_between_samples = 0.01 # time between two samples read (potentiometers)

        encoders = []
        potentiometers = []
        range_of_motion_joint = []

        average_encoder = [] # all encoder values collected at a sample position used for averaging the values of that position
        average_potentiometer = [] # all potentiometer values collected at a sample position used for averaging the values of that position
        d2r = math.pi / 180.0 # degrees to radians
        r2d = 180.0 / math.pi # radians to degrees

        slopes = []
        offsets = []
        average_offsets = []

        # Looking in XML assuming following tree structure
        # config > Robot> Actuator > AnalogIn > VoltsToPosSI > Scale = ____   or   Offset = ____
        xmlVoltsToPosSI = {}

        tree = ET.parse(filename)
        root = tree.getroot()
        robotFound = False
        stuffInRoot = root.getchildren()
        for index in range(len(stuffInRoot)):
            if stuffInRoot[index].tag == "Robot":
                currentRobot = stuffInRoot[index]
                if currentRobot.attrib["Name"] == self._robot_name:
                    self._serial_number = currentRobot.attrib["SN"]
                    xmlRobot = currentRobot
                    print("Succesfully found robot \"", currentRobot.attrib["Name"], "\", Serial number: ", self._serial_number, " in XML file")
                    robotFound = True
                else:
                    print("Found robot \"", currentRobot.attrib["Name"], "\", while looking for \"", self._robot_name, "\"")

        if robotFound == False:
            sys.exit("Robot tree could not be found in xml file")

        # look for all VoltsToPosSI
        stuffInRobot = xmlRobot.getchildren()
        for index in range(len(stuffInRobot)):
            child = stuffInRobot[index]
            if child.tag == "Actuator":
                actuatorId = int(child.attrib["ActuatorID"])
                stuffInActuator = child.getchildren()
                for subIndex in range(len(stuffInActuator)):
                    subChild = stuffInActuator[subIndex]
                    if subChild.tag == "AnalogIn":
                        stuffInAnalogIn = subChild.getchildren()
                        for subSubIndex in range(len(stuffInAnalogIn)):
                            subSubChild = stuffInAnalogIn[subSubIndex]
                            if subSubChild.tag == "VoltsToPosSI":
                                xmlVoltsToPosSI[actuatorId] = subSubChild

        # set joint limits and number of axis based on arm type, using robot name
        if ("").join(list(currentRobot.attrib["Name"])[:-1]) == "PSM": #checks to see if the robot being tested is a PSM
            arm_type = "PSM"
            lower_joint_limits = [-60.0 * d2r, -30.0 * d2r, 0.005, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r]
            upper_joint_limits = [ 60.0 * d2r,  30.0 * d2r, 0.235,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r]
            nb_axis = 7 #number of joints being tested
        elif currentRobot.attrib["Name"] == "MTML":
            arm_type = "MTM"
            lower_joint_limits = [-15.0 * d2r, -10.0 * d2r, -10.0 * d2r, -180.0 * d2r, -80.0 * d2r, -40.0 * d2r, -100.0 * d2r]
            upper_joint_limits = [ 35.0 * d2r,  20.0 * d2r,  10.0 * d2r,   80.0 * d2r, 160.0 * d2r,  40.0 * d2r,  100.0 * d2r]
            nb_axis = 7
        elif currentRobot.attrib["Name"] == "MTMR":
            arm_type = "MTM"
            lower_joint_limits = [-30.0 * d2r, -10.0 * d2r, -10.0 * d2r,  -80.0 * d2r, -80.0 * d2r, -40.0 * d2r, -100.0 * d2r]
            upper_joint_limits = [ 15.0 * d2r,  20.0 * d2r,  10.0 * d2r,  180.0 * d2r, 160.0 * d2r,  40.0 * d2r,  100.0 * d2r]
            nb_axis = 7
        elif currentRobot.attrib["Name"] == "ECM":
            arm_type = "ECM"
            lower_joint_limits = [-60.0 * d2r, -40.0 * d2r,  0.005, -80.0 * d2r]
            upper_joint_limits = [ 60.0 * d2r,  40.0 * d2r,  0.230,  80.0 * d2r]
            nb_axis = 4


        if arm_type == "PSM":
            this_arm = dvrk.psm(self._robot_name)
        else:
            this_arm = dvrk.arm(self._robot_name)


        # resize all arrays
        for axis in range(nb_axis):
            encoders.append([])
            offsets.append([])
            potentiometers.append([])
            average_encoder.append([])
            average_offsets.append([])
            average_potentiometer.append([])
            range_of_motion_joint.append(math.fabs(upper_joint_limits[axis] - lower_joint_limits[axis]))

        # Check that everything is working
        time.sleep(2.0) # to make sure some data has arrived
        if not self._data_received:
            print("It seems the console for ", self._robot_name, " is not started or is not publishing the IO topics")
            print("Make sure you use \"rosrun dvrk_robot dvrk_console_json\" with the -i option")
            sys.exit("Start the dvrk_console_json with the proper options first")

        print("The serial number found in the XML file is: ", self._serial_number)
        print("Make sure the dvrk_console_json is using the same configuration file.  Serial number can be found in GUI tab \"IO\".")
        ok = input("Press `c` and [enter] to continue\n")
        if ok != "c":
            sys.exit("Quitting")

        ######## scale calibration
        now = datetime.datetime.now()
        now_string = now.strftime("%Y-%m-%d-%H:%M")

        if calibrate == "scales":

            print("Calibrating scales using encoders as reference")

            # write all values to csv file
            csv_file_name = 'pot_calib_scales_' + self._robot_name + '-' + self._serial_number + '-' + now_string + '.csv'
            print("Values will be saved in: ", csv_file_name)
            f = open(csv_file_name, 'wb')
            writer = csv.writer(f)
            header = []
            for axis in range(nb_axis):
                header.append("potentiometer" + str(axis))
            for axis in range(nb_axis):
                header.append("encoder" + str(axis))
            writer.writerow(header)

            # messages
            input("To start with some initial values, you first need to \"home\" the robot.  When homed, press [enter]\n")

            if arm_type == "PSM":
                input("Since you are calibrating a PSM, make sure there is no tool inserted.  Please remove tool or calibration plate if any and press [enter]\n")
            if arm_type == "ECM":
                input("Since you are calibrating an ECM, remove the endoscope and press [enter]\n")
            input("The robot will make LARGE MOVEMENTS, please hit [enter] to continue once it is safe to proceed\n")

            for position in range(nb_joint_positions):
                # create joint goal
                joint_goal = []
                for axis in range(nb_axis):
                    joint_goal.append(lower_joint_limits[axis] + position * (range_of_motion_joint[axis] / nb_joint_positions))
                    average_encoder[axis] = []
                    average_potentiometer[axis] = []

                # move and sleep
                if arm_type == "PSM":
                    this_arm.move_jaw(joint_goal[6], blocking = False)
                    this_arm.move_joint(numpy.array(joint_goal[0:6]))
                else:
                    this_arm.move_joint(numpy.array(joint_goal))
                time.sleep(sleep_time_after_motion)

                # collect nb_samples_per_position at current position to compute average
                for sample in range(nb_samples_per_position):
                    for axis in range(nb_axis):
                        average_potentiometer[axis].append(self._last_potentiometers[axis])
                        average_encoder[axis].append(self._last_joints[axis])
                    # log data
                    writer.writerow(self._last_potentiometers + self._last_joints)
                    time.sleep(sleep_time_between_samples)
                    samples_so_far = samples_so_far + 1
                    sys.stdout.write('\rProgress %02.1f%%' % (float(samples_so_far) / float(total_samples) * 100.0))
                    sys.stdout.flush()

                # compute averages
                for axis in range(nb_axis):
                    potentiometers[axis].append(math.fsum(average_potentiometer[axis]) / nb_samples_per_position)
                    encoders[axis].append(math.fsum(average_encoder[axis]) / nb_samples_per_position)


            # at the end, return to home position
            if arm_type == "PSM":
                this_arm.move_jaw(0.0, blocking = False)
                this_arm.move_joint(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            elif arm_type == "MTM":
                this_arm.move_joint(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
            elif arm_type == "ECM":
                this_arm.move_joint(numpy.array([0.0, 0.0, 0.0, 0.0]))

            # close file
            f.close()


        ######## offset calibration
        if calibrate == "offsets":

            print("Calibrating offsets")

            # write all values to csv file
            csv_file_name = 'pot_calib_offsets_' + self._robot_name + '-' + self._serial_number + '-' + now_string + '.csv'
            print("Values will be saved in: ", csv_file_name)
            f = open(csv_file_name, 'wb')
            writer = csv.writer(f)
            header = []
            for axis in range(nb_axis):
                header.append("potentiometer" + str(axis))
            writer.writerow(header)

            # messages
            print("Please home AND power off the robot first.  Then hold/clamp your arm in zero position.")
            if arm_type == "PSM":
                print("For a PSM, you need to hold at least the last 4 joints in zero position.  If you don't have a way to constrain the first 3 joints, you can still just calibrate the last 4.  This program will ask you later if you want to save all PSM joint offsets");
            input("Press [enter] to continue\n")
            nb_samples = 10 * nb_samples_per_position
            for sample in range(nb_samples):
                for axis in range(nb_axis):
                    average_offsets[axis].append(self._last_potentiometers[axis] * r2d)
                writer.writerow(self._last_potentiometers)
                time.sleep(sleep_time_between_samples)
                sys.stdout.write('\rProgress %02.1f%%' % (float(sample) / float(nb_samples) * 100.0))
                sys.stdout.flush()
            for axis in range(nb_axis):
                offsets[axis] = (math.fsum(average_offsets[axis]) / (nb_samples) )

        print("")


        if calibrate == "scales":
            print("index | old scale  | new scale  | correction")
            for index in range(nb_axis):
                # find existing values
                oldScale = float(xmlVoltsToPosSI[index].attrib["Scale"])
                # compute new values
                correction = slope(encoders[index], potentiometers[index])
                newScale = oldScale / correction

                # display
                print(" %d    | % 04.6f | % 04.6f | % 04.6f" % (index, oldScale, newScale, correction))
                # replace values
                xmlVoltsToPosSI[index].attrib["Scale"] = str(newScale)

        if calibrate == "offsets":
            newOffsets = []
            print("index | old offset  | new offset | correction")
            for index in range(nb_axis):
                # find existing values
                oldOffset = float(xmlVoltsToPosSI[index].attrib["Offset"])
                # compute new values
                newOffsets.append(oldOffset - offsets[index])

                # display
                print(" %d    | % 04.6f | % 04.6f | % 04.6f " % (index, oldOffset, newOffsets[index], offsets[index]))

            if arm_type == "PSM":
                all = input("Do you want to save all joint offsets or just the last 4, press 'a' followed by [enter] to save all\n");
                if all == "a":
                    print("This program will save ALL new PSM offsets")
                    for axis in range(nb_axis):
                        # replace values
                        xmlVoltsToPosSI[axis].attrib["Offset"] = str(newOffsets[axis])
                else:
                    print("This program will only save the last 4 PSM offsets")
                    for axis in range(3, nb_axis):
                        # replace values
                        xmlVoltsToPosSI[axis].attrib["Offset"] = str(newOffsets[axis])
            else:
                for axis in range(nb_axis):
                    # replace values
                    xmlVoltsToPosSI[axis].attrib["Offset"] = str(newOffsets[axis])

        save = input("To save this in new file press 'y' followed by [enter]\n")
        if save == "y":
            tree.write(filename + "-new")
            print('Results saved in', filename + '-new. Restart your dVRK application with the new file!')
            print('To copy the new file over the existing one: cp', filename + '-new', filename)

if __name__ == '__main__':
    if (len(sys.argv) != 4):
        print(sys.argv[0] + ' requires three arguments, i.e. "scales"/"offsets", name of dVRK arm and file name.  Always start with scales calibration.')
    else:
        if (sys.argv[1] == 'offsets') or (sys.argv[1] == 'scales'):
            calibration = potentiometer_calibration(sys.argv[2])
            calibration.run(sys.argv[1], sys.argv[3])
        else:
            print(sys.argv[0] + ', first argument must be either scales or offsets.  You must start with the scale calibration')
