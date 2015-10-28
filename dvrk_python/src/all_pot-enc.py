from robot import *
import time
import math
import xml.etree.ElementTree as ET


pots = []

def pot_callback(data):
    pots[:] = data.position

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

    nb_samples = 10 # number of positions between limits
    number_of_points = 10 # number of values collected at each position

    sleep_time_after_motion = 1.0 # time after motion from position to position to allow potentiometers to stabilize

    nb_axis = 7 #number of joints being tested

    encoders = []
    potentiometers = []
    range_of_motion_joint = []

    average_encoder = []
    average_potentiometer = []

    lower_joint_limits = [-1.186, -0.837, 0.0, -2.61, -1.39, -0.871, 0]
    upper_joint_limits = [ 1.186,  0.837, 0.235, 2.61, 1.39, 0, 0.871]
    
    slopes = []
    actuators = []
    gain_list = []
    is_there_robot_in_xml = 0
    new_gains = []
    is_finished = False
    
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
                average_potentiometer[axis].append(pots[axis])
                average_encoder[axis].append(r.get_current_joint_position()[axis])
                time.sleep(.01)

        # compute averages
        for axis in range(0, nb_axis):
            potentiometers[axis].append(math.fsum(average_potentiometer[axis]) / number_of_points)
            encoders[axis].append(math.fsum(average_encoder[axis]) / number_of_points)

        print 'time left: ', ((nb_samples) * (sleep_time_after_motion + (number_of_points * 0.01))) - ((sample) * (sleep_time_after_motion + (number_of_points * 0.01)))

    for axis in range(0, nb_axis):
        print 'joint ', axis, ' slope: ', slope(encoders[axis], potentiometers[axis])
        slope_of_joint = slope(encoders[axis], potentiometers[axis])
        slopes.append(slope_of_joint)

    


#will have to modify xml file to divide current values by slopes
# location:   cd /home/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-daVinci/sawRobotIO1394-PSM2-32204.xml
# file name: sawRobotIO1394-PSM2-32204.xml
# Actuator > AnalogIn > VoltsToPosSI > Scale = ____
    


    tree = ET.parse('/home/neusman1/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-daVinci/sawRobotIO1394-PSM2-32204.xml')
    root = tree.getroot()
    stuffInRoot = root.getchildren()
    for children in range(0,len(stuffInRoot)):
        if stuffInRoot[children].tag == "Robot":
            xmlrobot = stuffInRoot[children]
        else:
            is_there_robot_in_xml += 1
            if is_there_robot_in_xml == len(stuffInRoot):
                print "Robot tree could not be found in xml file"
    stuffInRobot = xmlrobot.getchildren()
    for children in range(0,len(stuffInRobot)):
        if stuffInRobot[children].tag == "Actuator":
            actuators.append(stuffInRobot[children])
    #print actuators
    for children in range(0,len(actuators)):
        actuators[children] = actuators[children].getchildren()
        for grandchildren in range(0,len(actuators[children])):
            actuators[children][grandchildren] =  actuators[children][grandchildren].getchildren()
    #print actuators
    for gains in range(0,len(actuators)):
        gain = actuators[gains][2][1]
        #print gain.attrib['Scale']
        gain_list.append(gain.attrib["Scale"])
    print gain_list
    
    for ngains in range(0,len(gain_list)):
        new_gains.append( float(gain_list[ngains]) / float(slopes[ngains]) )
    print new_gains
    
    finish = raw_input("if these values seem correct, enter '/y'/, if not enter '/n'/ ")
    
    while is_finished == False:
        if finish == "y":
            for ngains in range(0,len(actuators)):
                actuators[ngains][2][1].set("Scale", new_gains[ngains])
                tree.write('/home/neusman1/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-daVinci/sawRobotIO1394-PSM2-32204-test.xml')
                #tree.write('sawRobotIO1394-PSM2-32204-test.xml')
            is_finished = True
            
        elif finish == "n":
            print "Calibration cancled"
            is_finished = True
        else:
            finish = raw_input("Not a correct value, please enter '/y'/ or '/n'/ ")
            is_finished = False
    print "Done"
    



if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        main(sys.argv[1])
