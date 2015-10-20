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
    #joint_number = int(raw_input('enter the joint you want to tested: '))
    number_of_points = 200 #int(raw_input('enter the number of points you want to tested: '))
    stop_time = 2 #float(raw_input('enter the stop time you want to tested: '))
    x_joint_0 = []
    y_joint_0 = []
    x_joint_1 = []
    y_joint_1 = []
    x_joint_2 = []
    y_joint_2 = []

    r.move_joint_list([1.186,0.837,0.0],[0,1,2])
    time.sleep(.5)
    first_extreme_joint_0 = r.get_current_joint_position()[0]
    first_extreme_joint_1 = r.get_current_joint_position()[1]
    first_extreme_joint_2 = r.get_current_joint_position()[2]
    time.sleep(.5)
    r.move_joint_list([-1.186,-0.837,0.235],[0,1,2])
    time.sleep(.5)
    second_extreme_joint_0 = r.get_current_joint_position()[0]
    second_extreme_joint_1 = r.get_current_joint_position()[1]
    second_extreme_joint_2 = r.get_current_joint_position()[2]
    time.sleep(.5)

    r.move_joint_list([0.0,0.0,0.1,0.0,0.0,0.0,0.0],[0,1,2,3,4,5,6])
    range_of_motion_joint_0 = (math.fabs(first_extreme_joint_0) + math.fabs(second_extreme_joint_0))
    range_of_motion_joint_1 = (math.fabs(first_extreme_joint_1) + math.fabs(second_extreme_joint_1))
    range_of_motion_joint_2 = (math.fabs(first_extreme_joint_2) + math.fabs(second_extreme_joint_2))

    for i in range(-50,50):

        move_amount_joint_0 =  (range_of_motion_joint_0 / 100) * i 
        move_amount_joint_1 =  (range_of_motion_joint_1 / 100) * i 
        move_amount_joint_2 =  ((range_of_motion_joint_2 / 100) * i ) + ( range_of_motion_joint_2 / 2 )

        r.move_joint_list([move_amount_joint_0],[0])
        r.move_joint_list([move_amount_joint_1],[1])
        r.move_joint_list([move_amount_joint_2],[2])
        
        time.sleep(stop_time)

        pot_points_joint_0 = []
        enc_points_joint_0 = []
        pot_points_joint_1 = []
        enc_points_joint_1 = []
        pot_points_joint_2 = []
        enc_points_joint_2 = []

        for c in range(0,number_of_points):
            pot_points_joint_0.append(pots[0])
            enc_points_joint_0.append(r.get_current_joint_position()[0])
            pot_points_joint_1.append(pots[1])
            enc_points_joint_1.append(r.get_current_joint_position()[1])
            pot_points_joint_2.append(pots[2])
            enc_points_joint_2.append(r.get_current_joint_position()[2])
            
            time.sleep(.01)
        pot_points_average_joint_0 = (math.fsum(pot_points_joint_0))/number_of_points
        enc_points_average_joint_0 = (math.fsum(enc_points_joint_0))/number_of_points
        pot_points_average_joint_1 = (math.fsum(pot_points_joint_1))/number_of_points
        enc_points_average_joint_1 = (math.fsum(enc_points_joint_1))/number_of_points
        pot_points_average_joint_2 = (math.fsum(pot_points_joint_2))/number_of_points
        enc_points_average_joint_2 = (math.fsum(enc_points_joint_2))/number_of_points

        x_joint_0.append(enc_points_average_joint_0)
        y_joint_0.append(pot_points_average_joint_0)
        x_joint_1.append(enc_points_average_joint_1)
        y_joint_1.append(pot_points_average_joint_1)
        x_joint_2.append(enc_points_average_joint_2)
        y_joint_2.append(pot_points_average_joint_2)

        print 'time left: ', ((100) * (stop_time + (number_of_points * .01))) - ((i + 50) * (stop_time + (number_of_points * .01)))      
        
    r.move_joint_list([0.0,0.0,0.1,0.0,0.0,0.0,0.0],[0,1,2,3,4,5,6])


    """
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
    """


    print "done"
    

    def slope(x,y):
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
        return slope
    print'joint 0 slope: ', slope(x_joint_0,y_joint_0)
    print'joint 1 slope: ', slope(x_joint_1,y_joint_1)
    print'joint 2 slope: ', slope(x_joint_2,y_joint_2)




#will have to modify xml file to divide current values by slopes 
# location:   cd ~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-daVinci/     
# file name: sawRobotIO1394-PSM3-28613.xml
# Actuator > AnalogIn > VoltsToPosSI > Scale = ____




if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        main(sys.argv[1])
