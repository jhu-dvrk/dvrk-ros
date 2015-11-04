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


def potCalibration(robotName,fileLocation):
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
    d2r = math.pi / 180.0
    r2d = 180.0 / math.pi
    lower_joint_limits = [-1.186, -0.837, 0.0,   -250.0 * d2r, -65.0 * d2r, -80.0 * d2r, 0.0 * d2r]
    upper_joint_limits = [ 1.186,  0.837, 0.235,  250.0 * d2r,  65.0 * d2r,  80.0 * d2r, 0.0 * d2r]

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
    
    
    raw_input("If you haven't already, hit [enter] and place a tool on the robot\n")
    r.home()
    raw_input("The robot will now start moving, please hit [enter] to continue once it is safe to proceed\n")
    
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
    
    print "Now calibrating offsets using calibration plate"
    r.move_joint_list([0.0,0.0,0.0,0.0],[3,4,5,6])
    r.shutdown()
    
    raw_input("Place the plate over the final four joints and hit [enter]\n")
    for data in range(0,number_of_points):
        for axis in range(3, nb_axis):
            average_offsets[axis].append(float(lastActuators[axis] * r2d))
        time.sleep(.01)
        for axis in range(0,3):
            average_offsets[axis].append(0.0)
    for axis in range(0,nb_axis):
        offsets[axis] = (math.fsum(average_offsets[axis]) / number_of_points)
        


    # Looking in XML assuming following tree structure 
    # config > Robot> Actuator > AnalogIn > VoltsToPosSI > Scale = ____   or   Offset = ____
    xmlVoltsToPosSI = {}

    tree = ET.parse(fileLocation)
    root = tree.getroot()
    robotFound = False
    stuffInRoot = root.getchildren()
    for index in range(0, len(stuffInRoot)):
        if stuffInRoot[index].tag == "Robot":
            currentRobot = stuffInRoot[index]
            if currentRobot.attrib["Name"] == robotName:
                xmlRobot = currentRobot
                print "Succesfully found robot \"", currentRobot.attrib["Name"], "\" in XML file"
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

    print "index | old scale  | new scale  | correction | old offset  | new offset"
    for index in range(0, nb_axis):
        # find existing values
        oldOffset = float(xmlVoltsToPosSI[index].attrib["Offset"])
        oldScale = float(xmlVoltsToPosSI[index].attrib["Scale"])
        # compute new values
        correction = slope(encoders[index], potentiometers[index])
        newScale = oldScale / correction
        newOffset = oldOffset - offsets[index]
       
        
        # display
        print " %d    | % 4.6f | % 4.6f | % 4.6f  | % 4.6f  | % 4.6f  " % (index, oldScale, newScale, correction, oldOffset, newOffset)
        # replace values
        xmlVoltsToPosSI[index].attrib["Scale"] = str(newScale)
        xmlVoltsToPosSI[index].attrib["Offset"] = str(newOffset)

    save = raw_input("To save this in new file press 'y' followed by [enter]\n")
    if save == "y":
        tree.write(fileLocation + "-new")
        print "Results saved in ", fileLocation + "-new"

if __name__ == '__main__':
    if (len(sys.argv) != 3):
        print sys.argv[0] + ' requires two arguments, i.e. name of dVRK arm and file name'
    else:
        potCalibration(sys.argv[1], sys.argv[2])
