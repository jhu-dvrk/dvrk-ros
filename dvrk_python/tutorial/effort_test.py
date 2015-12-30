from robot import *
import time

def effort_test(robotName):
    r=robot(robotName)
 
    while(r.get_desired_joint_effort()[2] < 1):
        print r.get_desired_joint_effort()[2]
        r.delta_move_cartesian_translation([0.0,0.0,-0.001])
        time.sleep(.3)
    print r.get_desired_joint_effort()[2]










if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        effort_test(sys.argv[1])
