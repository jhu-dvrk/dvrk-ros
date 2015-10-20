from robot import *
import time
import math

pots = []

def pot_callback(data):
    pots[:] = data.position

def main(robotName):
    r = robot(robotName)
    rospy.Subscriber('/dvrk/' + robotName +  '/io/analog_input_pos_si',
                     JointState, pot_callback)

    r.move_joint_list([0.0,0.0,0.1,0.0,0.0,0.0,0.0],[0,1,2,3,4,5,6])
    joint_number = int(raw_input('enter the joint you want to tested: '))
    number_of_points = int(raw_input('enter the number of points you want to tested: '))
    stop_time = float(raw_input('enter the stop time you want to tested: '))
    x = []
    y = []


    r.move_joint_list([1.186,0.837,0.0],[0,1,2])
    time.sleep(.5)
    first_extreme = r.get_current_joint_position()[joint_number]
    time.sleep(.5)
    r.move_joint_list([-1.186,-0.837,0.235],[0,1,2])
    time.sleep(.5)
    second_extreme = r.get_current_joint_position()[joint_number]
    time.sleep(.5)

    r.move_joint_list([0.0,0.0,0.1,0.0,0.0,0.0,0.0],[0,1,2,3,4,5,6])
    range_of_motion = (math.fabs(first_extreme) + math.fabs(second_extreme))
    


    for i in range(-50,50):
        if joint_number == 0 or joint_number == 1:
            move_amount =  (range_of_motion / 100) * i 
        if joint_number == 2:
            move_amount =  ((range_of_motion / 100) * i ) + ( range_of_motion / 2 )

        r.move_joint_list([move_amount],[joint_number])
        
        time.sleep(stop_time)
        pot_points = []
        enc_points = []
 
        print x
        print y
        for c in range(0,number_of_points):
            pot_points.append(pots[joint_number])
            enc_points.append(r.get_current_joint_position()[joint_number])
            time.sleep(.01)
        pot_points_average = (math.fsum(pot_points))/number_of_points
        enc_points_average = (math.fsum(enc_points))/number_of_points
        x.append(enc_points_average)
        y.append(pot_points_average)
        print 'time left: ', ((100) * (stop_time + (number_of_points * .01))) - ((i + 50) * (stop_time + (number_of_points * .01)))      
        
    r.move_joint_list([0.0,0.0,0.1,0.0,0.0,0.0,0.0],[0,1,2,3,4,5,6])

    xy = []
    for i in range(100):
        xy.append(x[i])
        xy.append(y[i])
    


    f = open('pot_v._enc_data.csv','w')
    
    f.write('Joint: ' + str(joint_number))
    f.write('\n')
    f.write('Number of Points: ' + str(number_of_points))
    f.write('\n')
    f.write('Stop Time: ' + str(stop_time))
    f.write('\n')
    f.write('encoder' ',' 'potentiometer')
    f.write('\n')
    for i in range(200):
        if i%2 == 0:
            f.write(str(xy[i]))
            f.write(',')
        elif i%2 == 1:
            f.write(str(xy[i]))
            f.write('\n')
    f.close

    print "done"
    
from robot import *
import time
import math

pots = []

def pot_callback(data):
    pots[:] = data.position

def main(robotName):
    r = robot(robotName)
    rospy.Subscriber('/dvrk/' + robotName +  '/io/analog_input_pos_si',
                     JointState, pot_callback)

    r.move_joint_list([0.0,0.0,0.1,0.0,0.0,0.0,0.0],[0,1,2,3,4,5,6])
    joint_number = int(raw_input('enter the joint you want to tested: '))
    number_of_points = int(raw_input('enter the number of points you want to tested: '))
    stop_time = float(raw_input('enter the stop time you want to tested: '))
    x = []
    y = []


    r.move_joint_list([1.186,0.837,0.0],[0,1,2])
    time.sleep(.5)
    first_extreme = r.get_current_joint_position()[joint_number]
    time.sleep(.5)
    r.move_joint_list([-1.186,-0.837,0.235],[0,1,2])
    time.sleep(.5)
    second_extreme = r.get_current_joint_position()[joint_number]
    time.sleep(.5)

    r.move_joint_list([0.0,0.0,0.1,0.0,0.0,0.0,0.0],[0,1,2,3,4,5,6])
    range_of_motion = (math.fabs(first_extreme) + math.fabs(second_extreme))
    


    for i in range(-50,50):
        if joint_number == 0 or joint_number == 1:
            move_amount =  (range_of_motion / 100) * i 
        if joint_number == 2:
            move_amount =  ((range_of_motion / 100) * i ) + ( range_of_motion / 2 )

        r.move_joint_list([move_amount],[joint_number])
        
        time.sleep(stop_time)
        pot_points = []
        enc_points = []
 
        print x
        print y
        for c in range(0,number_of_points):
            pot_points.append(pots[joint_number])
            enc_points.append(r.get_current_joint_position()[joint_number])
            time.sleep(.01)
        pot_points_average = (math.fsum(pot_points))/number_of_points
        enc_points_average = (math.fsum(enc_points))/number_of_points
        x.append(enc_points_average)
        y.append(pot_points_average)
        print 'time left: ', ((100) * (stop_time + (number_of_points * .01))) - ((i + 50) * (stop_time + (number_of_points * .01)))      
        
    r.move_joint_list([0.0,0.0,0.1,0.0,0.0,0.0,0.0],[0,1,2,3,4,5,6])

    xy = []
    for i in range(100):
        xy.append(x[i])
        xy.append(y[i])
    


    f = open('pot_v._enc_data.csv','w')
    
    f.write('Joint: ' + str(joint_number))
    f.write('\n')
    f.write('Number of Points: ' + str(number_of_points))
    f.write('\n')
    f.write('Stop Time: ' + str(stop_time))
    f.write('\n')
    f.write('encoder' ',' 'potentiometer')
    f.write('\n')
    for i in range(200):
        if i%2 == 0:
            f.write(str(xy[i]))
            f.write(',')
        elif i%2 == 1:
            f.write(str(xy[i]))
            f.write('\n')
    f.close

    print "done"
    

 
    a = []
    for i in range(0,len(x)):
        a.append(x[i]*y[i])
    sum_a = sum(a)
    final_a = (sum_a * len(x))
    #print final_a

    final_b = (sum(x)*sum(y))
    #print final_b

    c_squared = []
    for i in x:
    	c_squared.append(i**2)

    c_sum = sum(c_squared)
    final_c = (c_sum * len(x))
    #print final_c    

    final_d = (sum(x)**2)
    #print final_d

    slope = (final_a - final_b) / (final_c - final_d)
    print 'Slope: ', slope




    """
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

    """





if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        main(sys.argv[1])
