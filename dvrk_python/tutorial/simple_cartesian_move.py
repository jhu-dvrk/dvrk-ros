"""In this example we will take a look at the how to move in cartesian space. Please note we can move a different distance than the one specified. We will show how the following methods world:
 * delta_move_cartesian_translation()
 * delta_move_cartesian_rotation
 * move_cartesian_translation
 * move_cartesian_rotation

Lets take a look:"""

from robot import *
import math
import time

def cartesian_move(robotName):
    """Here is where we initialize the robot, in this demo we called the robot r, and we move the robot in castresian space accordingly.

    :param robotName: the name of the robot used """

    r = robot(robotName)
    print 'Starting position:'
    print r.get_desired_cartesian_position()

    list_1 = [0.01, 0.0, 0.0]
    r.delta_move_cartesian(list_1, True)
    print 'After translation along x by 0.01:'
    print r.get_desired_cartesian_position()

    list_2 = [0.0, -0.01,0.0]
    r.delta_move_cartesian(list_2, True)
    print 'After translation along y by 0.01:'
    print r.get_desired_cartesian_position()

    vec_3 = Vector(-0.01, 0.01, -0.01)
    r.delta_move_cartesian_translation(vec_3, True)
    print 'After translation along y by 0.01 and z by -0.01:'
    print r.get_desired_cartesian_position()

    rot_4 = Rotation()
    # PyKDL can create rotation matrices along axes
    rot_4.DoRotX(math.pi / 4.0) # rotate along x by 45 degrees
    r.delta_move_cartesian_rotation(rot_4, True)
    print 'After rotation along x by 45 degrees:'
    print r.get_desired_cartesian_position()

    rot_5 = Rotation()
    # PyKDL can create rotation matrices along axes
    rot_5.DoRotX(-(math.pi / 4.0)) # rotate along x by 45 degrees
    r.delta_move_cartesian_rotation(rot_5, True)
    print 'After rotation along x by -45 degrees:'
    print r.get_desired_cartesian_position()

    vec_6 = Vector(0.0,0.01,-0.15)
    r.move_cartesian_translation(vec_6)
    print 'After, moving to (0, 0.01,-0.15) '
    print r.get_desired_cartesian_position()

    vec_13 = Vector(0.0,0.01,-0.16)
    r.move_cartesian_translation(vec_13)
    print 'After, moving to (0, 0.01,0.15) '
    print r.get_desired_cartesian_position()


    # True means direct command to the robot, without trajectory generation
    # this means we can't give a goal too far away from current position AND
    # this command doesn't wait for the robot to move so we need to add a sleep
    list_7 = [0.01,0.0,-0.15]
    r.move_cartesian_translation(list_7, False)
    time.sleep(3)
    print 'After, move to (0.01,0,-0.15)'
    print r.get_desired_cartesian_position()

    vec_8 = Vector(0.0,0.01,-0.15)
    r.move_cartesian_translation(vec_8, False)
    time.sleep(3)#in seconds
    print 'After, move to (0,0.01,-0.15):'
    print r.get_desired_cartesian_position()

    rot_9 = Rotation()
    rot_9.DoRotY(-math.pi/6.0)
    r.move_cartesian_rotation(rot_9,True)
    print 'After, rotation in y by -pi/6:'
    print r.get_desired_cartesian_position()

if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run cartesian_move()"""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        cartesian_move(sys.argv[1])
