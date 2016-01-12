#This is Exercise1 - the goal of this project is to
#get the robot to trace out a square, with one edge being(0.0,0.0,-015)

#we are using the robot api, therefore we need to import it
from robot import *

#create a square of length `length`
def square(robotName):
    #inialize the robot
    r = robot(robotName)

    #starting position
    r.move_cartesian_translation([0.0,0.0,-0.15])
    #ask user for the length of the square
    input_length = raw_input('Enter the length of the square : ')
    length = float(input_length)

    #map out the square
    r.move_cartesian_translation([0.0,length, -0.15])
    r.move_cartesian_translation([length, length, -0.15])
    r.move_cartesian([length, 0.0, -0.15])
    r.move_cartesian([0.0, 0.0, -0.15])

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        #ask the user for which robot we are using
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        square(sys.argv[1])
