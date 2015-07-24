from robot import *
import math



def dictionary(robotName):
    r=robot(robotName)
    dict = {}
    dict['test'] = [('u'),('d')]
    dict['test2'] = [('r',90,60),('r',180,60),('r',270,60),('r',0,60)]
    dict['a'] = [('r',80,60),('r',280,30),('r',180,10),('r',0,10),('r',280,30)]
    dict['b'] = [('r',90,60),('r',350,30),('r',270,20),('r',190,30),('r',350,40),('r',270,23),('r',180,40)] 
    dict['c'] = [('u'),('r',0,35),('d'),('r',180,30),('r',108.43,15.8114),('r',90,30),('r',71.57,15.8114),('r',0,30)]
    dict['d'] = [('r',90,60),('r',340,30),('r',270,40),('r',200,30)]
    
    #dict['d'] = [('f',60),('r',110),('f',30),('r',70),('f',40),('r',70),('f',30)]
    #dict['e'] = [('u'),('r',90),('f',40),('r',180),('d'),('f',40),('r',90),('f',30),('r',90),('f',30),('r',180),('f',30),('r',90),('f',30),('r',90),('f',40)]
    #dict['f'] = [('f',30),('r',90),('f',30),('r',180),('f',30),('r',90),('f',30),('r',90),('f',40)]
    #dict['g'] = [('u'),('r',90),('f',40),('r',270),('f',35),('r',270),('f',12),('r',180),('d'),('f',12),('r',90),('f',35),('r',90),('f',40),('r',90),('f',60),('r',90),('f',40)]
    #dict['h'] = [('f',60),('r',180),('f',30),('r',270),('f',40),('r',270),('f',30),('r',180),('f',60)]
    #dict['i'] = [('r',90),('f',40),('r',180),('f',20),('r',90),('f',60),('r',90),('f',20),('r',180),('f',40)]
    #dict['j'] = [('u'),('f',16),('d'),('r',143.14),('f',15),('r',306.87),('f',12),('r',306.87),('f',15),('r',323.14),('f',40),('r',90),('f',12),('r',180),('f',24)]
    #dict['k'] = [('f',60),('r',180),(]
 

    #print dict['a'][0][0]
    r.move_cartesian([0.0,-0.1,-0.12])
    letter_number = 0

    while letter_number < .2:  

        cycle_number = 0  
        letter = raw_input('enter the letter you would like typed: ') 
        length_of_list = len(dict[letter])  

        while cycle_number < length_of_list: 
            if dict[letter][cycle_number][0] == 'r':                      # ('r',angle,distance)
                angle = math.radians(dict[letter][cycle_number][1])
                print 'a', angle
                length = (dict[letter][cycle_number][2])/1000.0
                print 'l', length

                # we want the robot to actual write something that we
                # can read and not something backwards
                x_position = math.cos(angle)*length
                y_position = math.sin(angle +180)*length

                r.delta_move_cartesian([x_position,y_position,0.0])

            elif dict[letter][cycle_number] == 'u':         #u = pen up
                r.delta_move_cartesian([0.0,0.0,-0.01])
            elif dict[letter][cycle_number] == 'd':         #d = pen down
                r.delta_move_cartesian([0.0,0.0,0.01])
            
            cycle_number += 1

        letter_number +=.05
        r.delta_move_cartesian([0.0,0.0,-0.01])
        r.move_cartesian_translation([0.0,-.10+letter_number,-0.12])
        print 'ln',letter_number
        print 'get', r.get_desired_cartesian_position()
        r.delta_move_cartesian([0.0,0.0,0.01])
        r.close_gripper()
       

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        dictionary(sys.argv[1])
