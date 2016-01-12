#This is Exercise3 - the goal of this project is to perform a simple
#cross stich motion using the da Vinci surgical robot. 

#we are using the robot api, therefore we need to import it
from robot import *

def stich(robotName):
    #inialize the robot
    r = robot(robotName)

    #starting position
    r.move_cartesian_translation([0.0,0.0,-0.15])
    #this program is entirely dependent on the idea of
    #pythagoream theorm
    x = 0.006
    y = 0.008

    #make the inital motion
    cross(x, y, r)
    r.delta_move_cartesian(xInput, 0.0, 0.0)
    #create the ending motion
    cross(-1*x, -1*y, r)

def cross(xInput, yInput, r):
    r.delta_move_cartesian_translation([xInput, yInput, 0.0])
    r.delta_move_cartesian_translation([-1 * xInput, yInput, 0.0])
    r.delta_move_cartesian_translation([xInput, yInput, 0.0])
    r.delta_move_cartesian_translation([-1 * xInput, yInput, 0.0])

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        #ask the user for which robot we are using
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        stich(sys.argv[1])
