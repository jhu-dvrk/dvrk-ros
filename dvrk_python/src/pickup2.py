from robot import *
import time
import math

def pickup2(robotname):
    r = robot(robotname)
    r.move_cartesian([0.0,0.0,-0.05])

    r.move_joint_list([0.0,0.0,0.15,0.0,0.0,0.0,0.0],[0,1,2,3,4,5,6])

    r.open_gripper()
    time.sleep(1)
    r.delta_move_joint_list([math.pi/4],[0])
    r.delta_move_joint_list([0.05],[2])
    r.close_gripper()
    time.sleep(1)
    r.delta_move_joint_list([-0.05],[2])    
    r.delta_move_joint_list([-math.pi/4],[0])
    r.open_gripper()
    time.sleep(1)

    r.delta_move_joint_list([-math.pi/4],[0])
    r.delta_move_joint_list([0.05],[2])
    r.close_gripper()
    time.sleep(1)
    r.delta_move_joint_list([-0.05],[2])    
    r.delta_move_joint_list([math.pi/4],[0])
    r.open_gripper()
    

    


if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        pickup2(sys.argv[1])
