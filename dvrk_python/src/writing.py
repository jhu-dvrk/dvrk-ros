from robot import *
import math



def dictionary(robotName):
    r=robot(robotName)
    dict = {}
    dict[' '] = [('u'),('d')]
    dict['test2'] = [('r',90,60),('r',180,60),('r',270,60),('r',0,60)]

    dict['a'] = [('r',80,60),('r',280,30),('r',180,10),('r',0,10),('r',280,30)]
    dict['b'] = [('r',90,60),('r',350,30),('r',270,20),('r',190,30),('r',350,40),('r',270,23),('r',180,40)] 
    dict['c'] = [('u'),('r',0,35),('d'),('r',180,30),('r',108.43,15.8114),('r',90,30),('r',71.57,15.8114),('r',0,30),('r',180,30),('r',251.57,15.8114),('r',270,30),('r',288.43,15.8114),('r',0,30)]
    dict['d'] = [('r',90,60),('r',340,30),('r',270,40),('r',200,30)]
    dict['e'] = [('u'),('r',0,40),('d'),('r',180,40),('r',90,30),('r',0,30),('r',180,30),('r',90,30),('r',0,40),('r',180,40),('r',270,60),('r',0,40)]
    dict['f'] = [('r',90,30),('r',0,30),('r',180,30),('r',90,30),('r',0,40),('r',180,40),('r',270,60)]
    dict['g'] = [('u'),('r',0,40),('r',90,35),('r',180,12),('d'),('r',0,12),('r',270,35),('r',180,40),('r',90,60),('r',0,40),('r',180,40),('r',270,60),('r',0,40)]
    dict['h'] = [('r',90,60),('r',270,30),('r',0,40),('r',90,30),('r',270,60)]
    dict['i'] = [('r',0,40),('r',180,20),('r',90,60),('r',180,20),('r',0,40),('r',180,20),('r',270,60),('r',0,20)]
    dict['j'] = [('u'),('r',90,16),('d'),('r',306.86,15),('r',0,12),('r',36.86,15),('r',90,40),('r',180,12),('r',0,24),('r',180,12),('r',270,40)]
    dict['k'] = [('r',90,60),('r',270,30),('r',45,40),('r',225,40),('r',315,40)]
    dict['l'] = [('r',90,60),('r',270,60),('r',0,40)]
    dict['m'] = [('r',90,60),('r',0,20),('r',270,60),('r',90,60),('r',0,20),('r',270,60)]
    dict['n'] = [('r',90,60),('r',303.69,72),('r',90,60),('r',270,60)]
    dict['o'] = [('r',90,60),('r',0,40),('r',270,60),('r',180,40),('r',0,40)]
    dict['p'] = [('r',90,60),('r',0,40),('r',270,30),('r',180,40),('r',270,30)]
    dict['q'] = [('r',90,60),('r',0,40),('r',270,60),('r',180,40),('r',0,40),('r',135,12),('r',315,24)]
    dict['r'] = [('r',90,60),('r',0,40),('r',270,30),('r',180,40),('r',315,45)]
    dict['s'] = [('r',0,40),('r',90,30),('r',180,40),('r',90,30),('r',0,40),('r',180,40),('r',270,30),('r',0,40),('r',270,30)]
    dict['t'] = [('u'),('r',0,20),('d'),('r',90,60),('r',180,20),('r',0,40),('r',180,20),('r',270,60)]
    dict['u'] = [('r',90,60),('r',270,60),('r',0,40),('r',90,60),('r',270,60)]
    dict['v'] = [('u'),('r',0,20),('d'),('r',108.44,63),('r',288.44,63),('r',71.56,63),('r',251.56,63)]
    dict['w'] = [('r',90,60),('r',270,60),('r',0,20),('r',90,40),('r',270,40),('r',0,20),('r',90,60),('r',270,60)]
    dict['x'] = [('r',56.31,72),('r',236.31,36),('r',123.69,36),('r',303.69,72)]
    dict['y'] = [('u'),('r',0,20),('d'),('r',90,30),('r',56.31,36),('r',236.31,36),('r',123.69,36),('r',303.69,36),('r',270,30)]
    dict['z'] = [('r',56.31,72),('r',180,40),('r',0,40),('r',236.31,72),('r',0,40)]


 

    start_x = -0.1
    r.move_cartesian([start_x,0.0,-0.12])
    rot_1 = Rotation.Identity()
    #rot_1 = Rotation(0.47,0.87,-0.17,0.80,-0.50,-0.34,-0.38,0.02,-0.92)
    r.move_cartesian_rotation(rot_1)
    letter_number = 0

    while letter_number < .2:  

        cycle_number = 0  
        letter = raw_input('enter the letter you would like typed: ').lower() 
        length_of_list = len(dict[letter])  

        while cycle_number < length_of_list: 
            if dict[letter][cycle_number][0] == 'r':                      # ('r',angle,distance)
                angle = math.radians(dict[letter][cycle_number][1])
                length = (dict[letter][cycle_number][2])/1000.0

                # get the letter to print in the correct direction
                x_position = math.cos(angle)*length #+ r.get_desired_cartesian_position().p.x()
                y_position = math.sin(angle)*length #+ r.get_desired_cartesian_position().p.y()

                r.delta_move_cartesian([x_position,y_position,0.0])
                #r.move_cartesian([x_position,y_position,-0.12])

            elif dict[letter][cycle_number] == 'u':         #u = pen up
                r.delta_move_cartesian([0.0,0.0,0.01])
            elif dict[letter][cycle_number] == 'd':         #d = pen down
                r.delta_move_cartesian([0.0,0.0,-0.01])

            cycle_number += 1

        letter_number +=.05
        r.delta_move_cartesian([0.0,0.0,0.01])
        r.move_cartesian_translation([start_x+letter_number,0.0,-0.11])
        r.delta_move_cartesian([0.0,0.0,-0.01])

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        dictionary(sys.argv[1])
