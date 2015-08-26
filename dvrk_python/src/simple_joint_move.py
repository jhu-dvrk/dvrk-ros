"""In this example we will take a look at the how to move in joint space. Please note we can move a different distance than the one specified. We will show how the following methods world:
 * delta_move_joint_translation()
 * delta_move_joint_rotation
 * move_joint_translation
 * move_joint_rotation

Lets take a look:"""

from robot import *
import time

def joint_move(robotName):
    """Here is where we initialize the robot, in this demo we called the robot r, and we move the robot in joint space accordingly.

    :param robotName: the name of the robot used """
    r = robot(robotName)
#look at size
    print 'After, move to position [0,0,0.15,0,0,0,0]:'
    r.move_joint_list([0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0], interpolate = True)
    print r.get_desired_joint_position()
 
    print 'After, move joint 4 by -0.01:'
    r.delta_move_joint_list([-0.01], [4], True)
    print r.get_desired_joint_position()

    # True means direct command to the robot, without trajectory generation
    # this means we can't give a goal too far away from current position AND
    # this command doesn't wait for the robot to move so we need to add a sleep
    print 'After, move joints 1 by 0.02 and joint 2 by 0.01:'
    r.delta_move_joint_list([0.02,0.01], [1,2], False)
    time.sleep(3)#in seconds
    print r.get_desired_joint_position()
 
    print 'After, move to position  [0.01,0.0,0.2,0.0,0.0,0.0,0.0]'
    arr1 = [0.01,0.0,0.2,0.0,0.0,0.0,0.0]
    r.move_joint_list(arr1, interpolate = True)
    print r.get_desired_joint_position()

    print 'After, move to position  [0.01,0.0,0.2,0.0,0.0,0.0,0.0]'
    r.move_joint_list(arr1, interpolate = True)
    time.sleep(3)
    print r.get_desired_joint_position()
 
    print 'After, move joint 4 to 0.01 and joint 3 to -0.02'
    r.move_joint_list([0.01,-0.02],[4,3], True)
    time.sleep(3)
    print r.get_desired_joint_position()

    arr3 = [0.0,0.0,0.2,0.0,0.0,0.0,0.0]
    r.move_joint_list(arr3, interpolate = False)
    time.sleep(3)
    print 'get_desired_joint_position() : \n', r.get_desired_joint_position()
   
    print 'After, move joint 0 to 0.0 and joint 3 by 0.1'
    r.move_joint_list([0.0,0.1],[0,3], True)
    print r.get_desired_joint_position()
   

if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run joint_move()"""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        joint_move(sys.argv[1])
