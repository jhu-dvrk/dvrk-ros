from robot import *

def picking(robotName):
    r = robot(robotName)
    
    r.move_cartesian_translation([0.0,0.0,-0.15])
    input_length = raw_input('Enter the length of the square : ')
    r.open_gripper


if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        picking(sys.argv[1])
