from robot import *

#create a square of length `length`
def square(robotName):
    r = robot(robotName)
    
    r.move_cartesian_translation([0.0,0.0,-0.15])
    #the length of the square
    input_length = raw_input('Enter the length of the square : ')
    length = float(input_length)

    #move in cartesian space
    r.move_cartesian_translation([0.0,length, -0.15])
    r.move_cartesian_translation([length, length, -0.15])
    r.move_cartesian([length, 0.0, -0.15])
    r.move_cartesian([0.0, 0.0, -0.15])

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        square(sys.argv[1])
