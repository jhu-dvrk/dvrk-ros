#!/usr/bin/env python

# Authors: Nick Eusman, Anton Deguet
# Date: 2015-09-01

# Todo:
# - add data saved to file for offsets/scales using xml filename to name the output file (e.g calib-offsets-sawRobotIO1394-PSM1-12345.csv)
# - test on ECM/MTM, use arm name to determine arm type, set number of axis and joint limits based on arm type
# - for offset calibration, instead of using average of all, eliminate outliers using std dev?  Test to see if more stable
# - test calibrating 3rd offset on PSM? 

import time
import rospy
import threading
import math
import sys

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

import xml.etree.ElementTree as ET

def slope(x, y):
    a = []
    for i in range(0,len(x)):
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
        self._last_potentiometers = []
        self._last_actuators = []
        self._robot_state = 'uninitialized'
        self._robot_state_event = threading.Event()
        self._goal_reached = False
        self._goal_reached_event = threading.Event()

    def pot_callback(self, data):
        self._last_potentiometers[:] = data.position

    def actuators_callback(self, data):
        self._last_actuators[:] = data.position

    def robot_state_callback(self, data):
        self._robot_state = data.data
        self._robot_state_event.set()

    def goal_reached_callback(self, data):
        self._goal_reached = data.data
        self._goal_reached_event.set()

    def set_position_goal_joint(self, goal):
        self._goal_reached_event.clear()
        self._goal_reached = False
        joint_state = JointState()
        joint_state.position[:] = goal
        self._set_position_goal_joint_publisher.publish(joint_state)
        self._goal_reached_event.wait(60) # 1 minute at most
        if not self._goal_reached:
            rospy.signal_shutdown('failed to reach goal')
            sys.exit(-1)

    def set_state_block(self, state, timeout = 60):
        self._robot_state_event.clear()
        self.set_robot_state.publish(state)
        self._robot_state_event.wait(timeout)
        if (self._robot_state != state):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state ' + state)
            rospy.signal_shutdown('failed to reach desired state')
            sys.exit(-1)

    def run(self, calibrate, filename):
        ros_namespace = '/dvrk/' + self._robot_name
        self.set_robot_state = rospy.Publisher(ros_namespace + '/set_robot_state', String, latch=True)
        self._set_position_goal_joint_publisher = rospy.Publisher(ros_namespace
                                                                  + '/set_position_goal_joint',
                                                                  JointState, latch = True)
        self._set_robot_state_publisher = rospy.Publisher(ros_namespace
                                                          + '/set_robot_state',
                                                          String, latch = True)
        rospy.Subscriber(ros_namespace + '/robot_state', String, self.robot_state_callback)
        rospy.Subscriber(ros_namespace + '/goal_reached', Bool, self.goal_reached_callback)
        rospy.Subscriber(ros_namespace +  '/io/analog_input_pos_si', JointState, self.pot_callback)
        rospy.Subscriber(ros_namespace +  '/io/actuator_position', JointState, self.actuators_callback)

        # create node
        rospy.init_node('dvrk_calibrate_potentiometers', anonymous = True)

        nb_joint_positions = 20 # number of positions between limits
        nb_samples_per_position = 500 # number of values collected at each position
        total_samples = nb_joint_positions * nb_samples_per_position
        samples_so_far = 0

        sleep_time_after_motion = 0.5 # time after motion from position to position to allow potentiometers to stabilize
        sleep_time_between_samples = 0.01 # time between two samples read (potentiometers)

        nb_axis = 7 #number of joints being tested

        encoders = []
        potentiometers = []
        range_of_motion_joint = []

        average_encoder = []
        average_potentiometer = []
        d2r = math.pi / 180.0
        r2d = 180.0 / math.pi
        lower_joint_limits = [-90.0 * d2r, -50.0 * d2r, 0.005, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r, -170.0 * d2r]
        upper_joint_limits = [ 90.0 * d2r,  50.0 * d2r, 0.235,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r,  170.0 * d2r]

        slopes = []
        offsets = []
        average_offsets = []

        for axis in range(0, nb_axis):
            encoders.append([])
            offsets.append([])
            potentiometers.append([])
            average_encoder.append([])
            average_offsets.append([])
            average_potentiometer.append([])
            range_of_motion_joint.append(math.fabs(upper_joint_limits[axis] - lower_joint_limits[axis]))


        if calibrate == "scales":
            print "Calibrating scales using encoders as reference"
            raw_input("To start with some initial values, you first need to \"home\" the robot.  When homed, press [enter]")
            raw_input("If you are calibrating a PSM, make sure there is no tool inserted.  Please remove tool or calibration plate if any and press [enter]")
            raw_input("The robot will make LARGE MOVEMENTS, please hit [enter] to continue once it is safe to proceed")
            
            # set in proper mode for joint control
            self.set_state_block('DVRK_POSITION_GOAL_JOINT')

            for position in range(0, nb_joint_positions):
                # create joint goal
                joint_goal = []
                for axis in range(0, nb_axis):
                    joint_goal.append(lower_joint_limits[axis] + position * (range_of_motion_joint[axis] / nb_joint_positions))
                    average_encoder[axis] = []
                    average_potentiometer[axis] = []

                # move and sleep
                self.set_position_goal_joint(joint_goal)
                time.sleep(sleep_time_after_motion)

                # collect nb_samples_per_position at current position to compute average
                for sample in range(0, nb_samples_per_position):
                    for axis in range(0, nb_axis):
                        average_potentiometer[axis].append(self._last_potentiometers[axis])
                        average_encoder[axis].append(self._last_actuators[axis])
                    time.sleep(sleep_time_between_samples)
                    samples_so_far = samples_so_far + 1
                    sys.stdout.write('\rProgress %02.1f%%' % (float(samples_so_far) / float(total_samples) * 100.0))
                    sys.stdout.flush()

                # compute averages
                for axis in range(0, nb_axis):
                    potentiometers[axis].append(math.fsum(average_potentiometer[axis]) / nb_samples_per_position)
                    encoders[axis].append(math.fsum(average_encoder[axis]) / nb_samples_per_position)

            # at the end, return to home position
            self.set_position_goal_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        if calibrate == "offsets":
            print "Calibrating offsets using calibration plate"
            raw_input("To start with some initial values, you first need to \"home\" the robot.  When homed, press [enter]")
            raw_input("The robot will now go in \"idle\" mode, i.e. turn PID off.  Press [enter]")
            self.set_state_block('DVRK_UNINITIALIZED')
            raw_input("Place the plate over the final four joints and hit [enter]")
            nb_samples = 10 * nb_samples_per_position
            for sample in range(0, nb_samples):
                for axis in range(3, nb_axis):
                    average_offsets[axis].append(self._last_potentiometers[axis] * r2d)
                time.sleep(sleep_time_between_samples)
                for axis in range(0, 3):
                    average_offsets[axis].append(0.0)
                sys.stdout.write('\rProgress %02.1f%%' % (float(sample) / float(nb_samples) * 100.0))
                sys.stdout.flush()
            for axis in range(0, nb_axis):
                offsets[axis] = (math.fsum(average_offsets[axis]) / (nb_samples) )

        print ""

        # Looking in XML assuming following tree structure
        # config > Robot> Actuator > AnalogIn > VoltsToPosSI > Scale = ____   or   Offset = ____
        xmlVoltsToPosSI = {}

        tree = ET.parse(filename)
        root = tree.getroot()
        robotFound = False
        stuffInRoot = root.getchildren()
        for index in range(0, len(stuffInRoot)):
            if stuffInRoot[index].tag == "Robot":
                currentRobot = stuffInRoot[index]
                if currentRobot.attrib["Name"] == self._robot_name:
                    xmlRobot = currentRobot
                    print "Succesfully found robot \"", currentRobot.attrib["Name"], "\" in XML file"
                    robotFound = True
                else:
                    print "Found robot \"", currentRobot.attrib["Name"], "\", while looking for \"", self._robot_name, "\""

        if robotFound == False:
            print "Robot tree could not be found in xml file"

        # look for all VoltsToPosSI
        stuffInRobot = xmlRobot.getchildren()
        for index in range(0, len(stuffInRobot)):
            child = stuffInRobot[index]
            if child.tag == "Actuator":
                actuatorId = int(child.attrib["ActuatorID"])
                stuffInActuator = child.getchildren()
                for subIndex in range(0, len(stuffInActuator)):
                    subChild = stuffInActuator[subIndex]
                    if subChild.tag == "AnalogIn":
                        stuffInAnalogIn = subChild.getchildren()
                        for subSubIndex in range(0, len(stuffInAnalogIn)):
                            subSubChild = stuffInAnalogIn[subSubIndex]
                            if subSubChild.tag == "VoltsToPosSI":
                                xmlVoltsToPosSI[actuatorId] = subSubChild

        if calibrate == "scales":
            print "index | old scale  | new scale  | correction"
            for index in range(0, nb_axis):
                # find existing values
                oldScale = float(xmlVoltsToPosSI[index].attrib["Scale"])
                # compute new values
                correction = slope(encoders[index], potentiometers[index])
                newScale = oldScale / correction

                # display
                print " %d    | % 04.6f | % 04.6f | % 04.6f" % (index, oldScale, newScale, correction)
                # replace values
                xmlVoltsToPosSI[index].attrib["Scale"] = str(newScale)

        if calibrate == "offsets":
            print "index | old offset  | new offset | correction"
            for index in range(0, nb_axis):
                # find existing values
                oldOffset = float(xmlVoltsToPosSI[index].attrib["Offset"])
                # compute new values
                newOffset = oldOffset - offsets[index]

                # display
                print " %d    | % 04.6f | % 04.6f | % 04.6f " % (index, oldOffset, newOffset, offsets[index])
                # replace values
                xmlVoltsToPosSI[index].attrib["Offset"] = str(newOffset)

        save = raw_input("To save this in new file press 'y' followed by [enter]\n")
        if save == "y":
            tree.write(filename + "-new")
            print 'Results saved in ', filename + '-new. Restart your dVRK application with the new file!'

if __name__ == '__main__':
    if (len(sys.argv) != 4):
        print sys.argv[0] + ' requires three arguments, i.e. "scales"/"offsets", name of dVRK arm and file name.  Always start with scales calibration.'
    else:
        if (sys.argv[1] == 'offsets') or (sys.argv[1] == 'scales'):
            calibration = potentiometer_calibration(sys.argv[2])
            calibration.run(sys.argv[1], sys.argv[3])
        else:
            print sys.argv[0] + ', first argument must be either scales or offsets.  You must start with the scale calibration'
