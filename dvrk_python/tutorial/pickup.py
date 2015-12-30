from robot import *

def pickup(robotname):
    r = robot(robotname)
    r.move_cartesian([0.0,0.0,-0.05])

    i = 0
    while i < 3:
        r.move_cartesian([-0.025+(i*.025),0.0,-0.05])
        r.delta_move_cartesian([0.0,0.05,0.0])
        r.open_gripper()
        r.delta_move_cartesian([0.0,0.0,-0.01])
        r.close_gripper()
        time.sleep(3)
        r.delta_move_cartesian([0.0,0.0,0.01])        
        r.move_cartesian([0.0,0.0,-0.05])
        r.open_gripper()
        i+=1
    r.move_cartesian([0.0,0.0,-0.05])


if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        pickup(sys.argv[1])
