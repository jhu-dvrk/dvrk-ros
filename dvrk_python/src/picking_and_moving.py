from robot import *
import math
import time
#in this program the end position will always be 0.1 meters in the
#positive x diection of the start position
def picking_and_moving(robotName, start):
    r = robot(robotName)

    r.move_cartesian_translation(start)
    r.delta_move_cartesian_translation([0.0,0.0,-0.1])
    r.open_gripper()
    time.sleep(2)
    r.close_gripper()
    
    time.sleep(2)


    delta_move_cartesian_translation([0.01,0.0,0.0])
    print 'end position : ', r.get_desired_cartesian_position()

if __name__ == '__main__':
    if (len(sys.argv) != 4):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        polygon(sys.argv[1], int(sys.argv[2]), float(sys.argv[3]))

