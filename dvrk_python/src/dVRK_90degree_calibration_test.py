from robot import *
import numpy
from copy import deepcopy
import time
import math



def dVRK_90degree_calibariton_test(robotName):
    r=robot(robotName)


    """
    a = numpy.array = ([])
    b = numpy.array = ([])
    
    for i in range(-45,45):

        r.move_joint_list([i * math.pi / 180],[1])
        print i * math.pi / 180
        time.sleep(.2)
        a = numpy.hstack ((a, [r.get_current_joint_position()[1]] * len(a) ))
        print a
        #b = (a[i] * 2)

    print numpy.polyfit(a,b,1)

"""

    x = []
    y = []
    r.move_joint_list([0.0,0.0,0.1,0.0,0.0,0.0,0.0],[0,1,2,3,4,5,6])
    joint_number = int(raw_input('enter the joint you want to test: '))
    desired_or_current = raw_input('would you like to test desired or current: ').lower()
    
    if joint_number == 0 or joint_number == 1 or joint_number == 3 or joint_number == 4 or joint_number == 5 or joint_number == 6:
        
        if desired_or_current == "desired":

            for i in range(-45,46):

                r.move_joint_list([i * math.pi / 180],[joint_number])
                print i * math.pi / 180
                time.sleep(.2)
                x.append(r.get_desired_joint_position()[joint_number])
                print x
                y.append(r.get_desired_joint_position()[joint_number] * 2)
    
            
        elif desired_or_current == "current":

            for i in range(-45,46):

                r.move_joint_list([i * math.pi / 180],[joint_number])
                print i * math.pi / 180
                time.sleep(.2)
                x.append(r.get_current_joint_position()[joint_number])
                print x
                y.append(r.get_current_joint_position()[joint_number] * 2)

        else:
            print "invalid input"

    elif joint_number == 2:

        if desired_or_current == "desired":

            for i in range(-45,46):

                r.move_joint_list([i * math.pi / 180],[joint_number])
                print i * math.pi / 180
                time.sleep(.2)
                x.append(r.get_desired_joint_position()[joint_number])
                print x
                y.append(r.get_desired_joint_position()[joint_number] * 2)
    
            
        elif desired_or_current == "current":

            for i in range(-45,46):

                r.move_joint_list([i * math.pi / 180],[joint_number])
                print i * math.pi / 180
                time.sleep(.2)
                x.append(r.get_current_joint_position()[joint_number])
                print x
                y.append(r.get_current_joint_position()[joint_number] * 2)

        else:
            print "invalid input"
    
    else:
        print"that isn't a joint number from 0-6"


    x_mean = sum(x)/len(x)
    y_mean = sum(y)/len(y)
    a = x
    b = y
    x_points = x
    y_points = y
    a[:] = [x - x_mean for x in a]
    b[:] = [y - y_mean for y in b]
    sum_a = sum(a)
    sum_b = sum(b)
    ab = []
    for i in range(0,len(a)):
        ab.append(a[i]*b[i])
    a_squared = a
    b_squared = b
    a_squared[:] =[a ** 2 for a in a_squared]
    b_squared[:] =[b ** 2 for b in b_squared]
    sum_ab = sum(ab)
    sum_a_squared = sum(a_squared)
    sum_b_squared = sum(b_squared)
    correlation = sum_ab/(math.sqrt((sum_a_squared)*(sum_b_squared)))
    print 'correlation',"%.16f" % correlation
    x_standard_deviation = math.sqrt((sum_a ** 2)/(len(x_points)-1))
    y_standard_deviation = math.sqrt((sum_b ** 2)/(len(y_points)-1))
    regression_slope = correlation * ( y_standard_deviation / x_standard_deviation )  
    print 'regression slope',"%.16f" % regression_slope


    


    


if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        dVRK_90degree_calibariton_test(sys.argv[1])
