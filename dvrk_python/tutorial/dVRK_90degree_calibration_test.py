from robot import *
import numpy
from copy import deepcopy
import time
import math



def dVRK_90degree_calibariton_test(robotName):
    r=robot(robotName)


    x = []
    y = []
    r.move_joint_list([0.0,0.0,0.1,0.0,0.0,0.0,0.0],[0,1,2,3,4,5,6])
    joint_number = int(raw_input('enter the joint you want to test: '))
    
    if joint_number == 0 or joint_number == 1 or joint_number == 3 or joint_number == 4 or joint_number == 5 or joint_number == 6:
        
      

        for i in range(-45,46):

            r.move_joint_list([i * math.pi / 180],[joint_number])
            time.sleep(2)
            x.append(r.get_current_joint_position()[joint_number])
            y.append(r.get_desired_joint_position()[joint_number])
            print x

    elif joint_number == 2:

     
        for i in range(-45,46):
            r.move_joint_list([(i * 0.001) + 0.15],[joint_number])
            time.sleep(2)
            x.append(r.get_current_joint_position()[joint_number])
            y.append(r.get_desired_joint_position()[joint_number])
            print x

    else:
        print "that isn't a joint number from 0-6"


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
    print 'correlation: ',"%.16f" % correlation
    x_standard_deviation = math.sqrt((sum_a ** 2)/(len(x_points)-1))
    print 'x standard deviation: ',"%.16f" % x_standard_deviation
    y_standard_deviation = math.sqrt((sum_b ** 2)/(len(y_points)-1))
    print 'y standard deviation: ',"%.16f" % y_standard_deviation    
    regression_slope = correlation * ( y_standard_deviation / x_standard_deviation )  
    print 'regression slope: ',"%.16f" % regression_slope


    


    


if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        dVRK_90degree_calibariton_test(sys.argv[1])
