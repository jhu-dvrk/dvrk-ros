#!/usr/bin/env python

# Authors: Nick Eusman, Anton Deguet
# Date: 2015-09-24
# Copyright JHU 2015-2023

import time
import math
import sys
import csv
import datetime
import numpy
import argparse
import os.path
import os

import dvrk
import crtk

import xml.etree.ElementTree as ET

# for actuator_to_joint_position
import cisst_msgs.srv

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

    # class to contain measured_js
    class __sensor:
        def __init__(self, ral, expected_interval):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval)
            self.__crtk_utils.add_measured_js()


    def __init__(self, ral, arm_name, expected_interval = 0.01):
        self.serial_number = ""
        self.expected_interval = expected_interval
        self.ros_namespace = arm_name
        # Create the dVRK python ROS client
        self.arm = dvrk.arm(ral = ral, arm_name = arm_name, expected_interval = expected_interval)
        self.potentiometers = self.__sensor(ral.create_child(arm_name + '/io/pot'), expected_interval)
        self.encoders = self.__sensor(ral.create_child(arm_name + '/io/actuator'), expected_interval)
        self.arm.check_connections()


    def run(self, calibration_type, filename):
        nb_joint_positions = 20 # number of positions between limits
        nb_samples_per_position = 250 # number of values collected at each position
        total_samples = nb_joint_positions * nb_samples_per_position
        samples_so_far = 0

        sleep_time_after_motion = 0.5 # time after motion from position to position to allow potentiometers to stabilize
        sleep_time_between_samples = self.expected_interval * 2.0 # time between two samples read (potentiometers)

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

        # Make sure file exists
        if not os.path.exists(filename):
            print('Config file "{}" not found'.format(filename))
            return

        tree = ET.parse(filename)
        root = tree.getroot()

        # Find Robot in config file and make sure name matches
        xpath_search_results = root.findall('./Robot')
        if len(xpath_search_results) == 1:
            xmlRobot = xpath_search_results[0]
        else:
            print('Can\'t find "Robot" in configuration file')
            return

        if xmlRobot.get('Name') == self.ros_namespace:
            self.serial_number = xmlRobot.get('SN')
            print('Successfully found robot "{}", serial number {} in XML file'.format(self.ros_namespace, self.serial_number))
            robotFound = True
        else:
            print('Found robot "{}" while looking for "{}", make sure you\'re using the correct configuration file!'.format(xmlRobot.get('Name'), self.ros_namespace))
            return

        # Look for all actuators/VoltsToPosSI
        xpath_search_results = root.findall('./Robot/Actuator')
        for actuator in xpath_search_results:
            actuatorId = int(actuator.get('ActuatorID'))
            voltsToPosSI = actuator.find('./AnalogIn/VoltsToPosSI')
            xmlVoltsToPosSI[actuatorId] = voltsToPosSI

        # set joint limits and number of axis based on arm type, using robot name
        if ('').join(list(self.ros_namespace)[:-1]) == 'PSM': #checks to see if the robot being tested is a PSM
            arm_type = 'PSM'
            lower_joint_limits = [-60.0 * d2r, -30.0 * d2r, 0.005, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r]
            upper_joint_limits = [ 60.0 * d2r,  30.0 * d2r, 0.235,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r]
            nb_axis = 7
        elif self.ros_namespace == 'MTML':
            arm_type = 'MTM'
            lower_joint_limits = [-15.0 * d2r, -10.0 * d2r, -10.0 * d2r, -180.0 * d2r, -80.0 * d2r, -40.0 * d2r, -100.0 * d2r]
            upper_joint_limits = [ 35.0 * d2r,  20.0 * d2r,  10.0 * d2r,   80.0 * d2r, 160.0 * d2r,  40.0 * d2r,  100.0 * d2r]
            nb_axis = 7
        elif self.ros_namespace == 'MTMR':
            arm_type = 'MTM'
            lower_joint_limits = [-30.0 * d2r, -10.0 * d2r, -10.0 * d2r,  -80.0 * d2r, -80.0 * d2r, -40.0 * d2r, -100.0 * d2r]
            upper_joint_limits = [ 15.0 * d2r,  20.0 * d2r,  10.0 * d2r,  180.0 * d2r, 160.0 * d2r,  40.0 * d2r,  100.0 * d2r]
            nb_axis = 7
        elif self.ros_namespace == 'ECM':
            arm_type = 'ECM'
            lower_joint_limits = [-60.0 * d2r, -40.0 * d2r,  0.005, -80.0 * d2r]
            upper_joint_limits = [ 60.0 * d2r,  40.0 * d2r,  0.230,  80.0 * d2r]
            nb_axis = 4

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
        try:
            time.sleep(20.0 * self.expected_interval)
            self.potentiometers.measured_jp()
        except:
            print('It seems the console for {} is not started or is not publishing the IO topics'.format(self.ros_namespace))
            print('Make sure you use "ros2 run dvrk_robot dvrk_console_json" with -i ros-io-{}.json'.format(self.ros_namespace))
            print('Start the dvrk_console_json with the proper options first')
            return


        print('The serial number found in the XML file is: {}'.format(self.serial_number))
        print('Make sure the dvrk_console_json is using the same configuration file.  Serial number can be found in GUI tab "IO".')
        ok = input('Press "c" and [enter] to continue\n')
        if ok != 'c':
            print('Quitting')
            return

        ######## scale calibration
        now = datetime.datetime.now()
        now_string = now.strftime('%Y-%m-%d-%H:%M')

        if calibration_type == 'scales':

            print('Calibrating scales using encoders as reference')

            # write all values to csv file
            csv_file_name = 'pot_calib_scales_' + self.ros_namespace + '-' + self.serial_number + '-' + now_string + '.csv'
            print('Values will be saved in: {}'.format(csv_file_name))
            f = open(csv_file_name, 'w')
            writer = csv.writer(f)
            header = []
            for axis in range(nb_axis):
                header.append('potentiometer' + str(axis))
            for axis in range(nb_axis):
                header.append('encoder' + str(axis))
            writer.writerow(header)

            # messages
            input('To start with some initial values, you first need to "home" the robot.  When homed, press [enter]\n')

            if arm_type == 'PSM':
                input('Since you are calibrating a PSM, make sure there is no instrument inserted.  Please remove instrument, sterile adapter or calibration plate if any and press [enter]\n')
            if arm_type == 'ECM':
                input('Since you are calibrating an ECM, remove the endoscope and press [enter]\n')
            input('The robot will make LARGE MOVEMENTS, please hit [enter] to continue once it is safe to proceed\n')

            for position in range(nb_joint_positions):
                # create joint goal
                joint_goal = []
                for axis in range(nb_axis):
                    joint_goal.append(lower_joint_limits[axis] + position * (range_of_motion_joint[axis] / nb_joint_positions))
                    average_encoder[axis] = []
                    average_potentiometer[axis] = []

                # move and sleep
                self.arm.move_jp(numpy.array(joint_goal)).wait()
                time.sleep(sleep_time_after_motion)

                # collect nb_samples_per_position at current position to compute average
                for sample in range(nb_samples_per_position):
                    last_pot = self.potentiometers.measured_jp()
                    last_enc = self.encoders.measured_jp()
                    for axis in range(nb_axis):
                        average_potentiometer[axis].append(last_pot[axis])
                        average_encoder[axis].append(last_enc[axis])
                    # log data
                    writer.writerow(last_pot + last_enc)
                    time.sleep(sleep_time_between_samples)
                    samples_so_far = samples_so_far + 1
                    sys.stdout.write('\rProgress %02.1f%%' % (float(samples_so_far) / float(total_samples) * 100.0))
                    sys.stdout.flush()

                # compute averages
                for axis in range(nb_axis):
                    potentiometers[axis].append(math.fsum(average_potentiometer[axis]) / nb_samples_per_position)
                    encoders[axis].append(math.fsum(average_encoder[axis]) / nb_samples_per_position)


            # at the end, return to home position
            if arm_type == 'PSM' or arm_type == 'MTM':
                self.arm.move_jp(numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])).wait()
            elif arm_type == 'ECM':
                self.arm.move_jp(numpy.array([0.0, 0.0, 0.0, 0.0])).wait()

            # close file
            f.close()


        ######## offset calibration
        if calibration_type == 'offsets':

            print('Calibrating offsets')

            # write all values to csv file
            csv_file_name = 'pot_calib_offsets_' + self.ros_namespace + '-' + self.serial_number + '-' + now_string + '.csv'
            print('Values will be saved in: ', csv_file_name)
            f = open(csv_file_name, 'w')
            writer = csv.writer(f)
            header = []
            for axis in range(nb_axis):
                header.append('potentiometer' + str(axis))
            writer.writerow(header)

            # messages
            print('Please home the robot first.  Then you can either power off the robot and hold/clamp your arm in zero position or you can keep the robot on and move it to the physical zero position (e.g. using the arm widget/direct-control)')
            if arm_type == 'PSM':
                print('For a PSM, you need to hold at least the last 4 joints in zero position.  If you don\'t have a way to constrain the first 3 joints, you can still just calibrate the last 4.  This program will ask you later if you want to save all PSM joint offsets');
            input('Press [enter] to continue\n')
            nb_samples = 2 * nb_samples_per_position
            for sample in range(nb_samples):
                last_pot = self.potentiometers.measured_jp()
                for axis in range(nb_axis):
                    average_offsets[axis].append(last_pot[axis] * r2d)
                writer.writerow(last_pot)
                time.sleep(sleep_time_between_samples)
                sys.stdout.write('\rProgress %02.1f%%' % (float(sample) / float(nb_samples) * 100.0))
                sys.stdout.flush()
            for axis in range(nb_axis):
                offsets[axis] = (math.fsum(average_offsets[axis]) / (nb_samples) )

        print('')


        if calibration_type == 'scales':
            print('index | old scale  | new scale  | correction')
            for index in range(nb_axis):
                # find existing values
                oldScale = float(xmlVoltsToPosSI[index].get('Scale'))
                # compute new values
                correction = slope(encoders[index], potentiometers[index])
                newScale = oldScale / correction

                # display
                print(' %d    | % 04.6f | % 04.6f | % 04.6f' % (index, oldScale, newScale, correction))
                # replace values
                xmlVoltsToPosSI[index].set('Scale', str(newScale))

        if calibration_type == 'offsets':
            # convert offsets to joint space
            a_to_j_service = ral.service_client(self.ros_namespace + '/actuator_to_joint_position', cisst_msgs.srv.ConvertFloat64Array)
            request = cisst_msgs.srv.ConvertFloat64ArrayRequest()
            request.input = offsets
            response = a_to_j_service(request)
            offsets = response.output

            newOffsets = []
            print('index | old offset  | new offset | correction')
            for index in range(nb_axis):
                # find existing values
                oldOffset = float(xmlVoltsToPosSI[index].get('Offset'))
                # compute new values
                newOffsets.append(oldOffset - offsets[index])

                # display
                print(' %d    | % 04.6f | % 04.6f | % 04.6f ' % (index, oldOffset, newOffsets[index], offsets[index]))

            if arm_type == 'PSM':
                all = input('Do you want to save all joint offsets or just the last 4, press "a" followed by [enter] to save all\n');
                if all == 'a':
                    print('This program will save ALL new PSM offsets')
                    for axis in range(nb_axis):
                        # replace values
                        xmlVoltsToPosSI[axis].set('Offset', str(newOffsets[axis]))
                else:
                    print('This program will only save the last 4 PSM offsets')
                    for axis in range(3, nb_axis):
                        # replace values
                        xmlVoltsToPosSI[axis].set('Offset', str(newOffsets[axis]))
            else:
                for axis in range(nb_axis):
                    # replace values
                    xmlVoltsToPosSI[axis].set('Offset', str(newOffsets[axis]))

        save = input('To save this, press "y" followed by [enter]\n')
        if save == 'y':
            os.rename(filename, filename + '-backup')
            tree.write(filename)
            print('Results saved in {}. Restart your dVRK application to use the new file!'.format(filename))
            print('Old file saved as {}-backup.'.format(filename))
        else:
            print('Results not saved!')


if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--type', type=str, required=True,
                        choices=['scales', 'offsets'],
                        help = 'use either "scales" or "offsets"')
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-c', '--config', type=str, required=True,
                        help = 'arm IO config file, i.e. something like sawRobotIO1394-xwz-12345.xml')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_calibrate_potentiometers')
    application = potentiometer_calibration(ral, args.arm)
    ral.spin_and_execute(application.run, args.type, args.config)
