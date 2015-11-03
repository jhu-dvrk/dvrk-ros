from robot import *
import time
import math
import xml.etree.ElementTree as ET


lastPotentiometers = []
lastActuators = []

def pot_callback(data):
    lastPotentiometers[:] = data.position

def actuatorsCallback(data):
    lastActuators[:] = data.position

def slope(x,y):
    a = []
    for i in range(0,len(x)):
        a.append(x[i]*y[i])
    sum_a = sum(a)
    final_a = (sum_a * len(x))
    final_b = (sum(x)*sum(y))

    c_squared = []
    for i in x:
        c_squared.append(i**2)

    c_sum = sum(c_squared)
    final_c = (c_sum * len(x))

    final_d = (sum(x)**2)

    slope = (final_a - final_b) / (final_c - final_d)
    return slope


def main(robotName):
    r = robot(robotName)
    rospy.Subscriber('/dvrk/' + robotName +  '/io/analog_input_pos_si',
                     JointState, pot_callback)
    rospy.Subscriber('/dvrk/' + robotName +  '/io/actuator_position',
                     JointState, actuatorsCallback)

    nb_samples = 20 # number of positions between limits
    number_of_points = 100 # number of values collected at each position

    sleep_time_after_motion = 0.5 # time after motion from position to position to allow potentiometers to stabilize

    nb_axis = 7 #number of joints being tested

    encoders = []
    potentiometers = []
    range_of_motion_joint = []

    average_encoder = []
    average_potentiometer = []

    lower_joint_limits = [-1.186, -0.837, 0.0, -2.61, -1.39, -0.871, 0]
    upper_joint_limits = [ 1.186,  0.837, 0.235, 2.61, 1.39, 0, 0.871]

    slopes = []

    for axis in range(0, nb_axis):
        encoders.append([])
        potentiometers.append([])
        average_encoder.append([])
        average_potentiometer.append([])
        range_of_motion_joint.append(math.fabs(upper_joint_limits[axis] - lower_joint_limits[axis]))

    for sample in range(0, nb_samples):
        # create joint goal
        joint_goal = []
        for axis in range(0, nb_axis):
            joint_goal.append(lower_joint_limits[axis] + sample * (range_of_motion_joint[axis] / nb_samples))
            average_encoder[axis] = []
            average_potentiometer[axis] = []

        # move and sleep
        r.move_joint_list(joint_goal, range(0, nb_axis))
        time.sleep(sleep_time_after_motion)

        # collect number_of_points at current position to compute average
        for data in range(0, number_of_points):
            for axis in range(0, nb_axis):
                average_potentiometer[axis].append(lastPotentiometers[axis])
                average_encoder[axis].append(lastActuators[axis])
            time.sleep(.01)

        # compute averages
        for axis in range(0, nb_axis):
            potentiometers[axis].append(math.fsum(average_potentiometer[axis]) / number_of_points)
            encoders[axis].append(math.fsum(average_encoder[axis]) / number_of_points)

        print 'time left: ', ((nb_samples) * (sleep_time_after_motion + (number_of_points * 0.01))) - ((sample) * (sleep_time_after_motion + (number_of_points * 0.01)))

#will have to modify xml file to divide current values by slopes
# location:   cd /home/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-daVinci/sawRobotIO1394-PSM2-32204.xml
# file name: sawRobotIO1394-PSM2-32204.xml
# Actuator > AnalogIn > VoltsToPosSI > Scale = ____


    xmlVoltsToPosSI = {}

    tree = ET.parse('/home/neusman1/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-daVinci/sawRobotIO1394-PSM2-32204.xml')
    root = tree.getroot()
    robotFound = False
    stuffInRoot = root.getchildren()
    for index in range(0, len(stuffInRoot)):
        if stuffInRoot[index].tag == "Robot":
            currentRobot = stuffInRoot[index]
            if currentRobot.attrib["Name"] == robotName:
                xmlRobot = currentRobot
                robotFound = True
            else:
                print "Found robot \"", currentRobot.attrib["Name"], "\", while looking for \"", robotName, "\""

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

    print "index | old scale  | new scale  | correction | old offset"
    for index in range(0, nb_axis):
        # find existing values
        oldOffset = float(xmlVoltsToPosSI[index].attrib["Offset"])
        oldScale = float(xmlVoltsToPosSI[index].attrib["Scale"])
        # compute new values
        correction = slope(encoders[index], potentiometers[index])
        newScale = oldScale / correction
        # display
        print " %d    | % 4.6f | % 4.6f | % 4.6f  | % 4.6f " % (index, oldScale, newScale, correction, oldOffset)
        # replace values
        xmlVoltsToPosSI[index].attrib["Scale"] = str(newScale)

    save = raw_input("if these values seem correct, enter y, if not enter n ")
    if save == "y":
        tree.write('/home/neusman1/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-daVinci/sawRobotIO1394-PSM2-32204-test.xml')

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        main(sys.argv[1])
