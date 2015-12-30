"""In this class we will take a look at all of the getters that are avilable in the robot api. We will show how the following methods work:
 * get_desired_cartesian_position()
 * get_current_cartesian_position()
 * get_desired_joint_position()
 * get_current_joint_position()

Lets take a look:"""

from robot import *

def getters(robotName):
    """Here is where we initialize the robot, in this demo we called the robot r, and show how to use each methods.

    :param robotName: the name of the robot used """
    r = robot(robotName)

    #get_desired_cartesian_position() will return the desired positon
    #of the robot in terms of cartesian coordinates
    print 'get_desired_cartesian_position() : \n', r.get_desired_cartesian_position()

    #get_current_cartesian_position() will return the current positon
    #of the robot in terms of cartesian coordinates
    print 'get_current_cartesian_position() : \n', r.get_current_cartesian_position()

    #get_joint_number() will return the number of joints on the
    #specific arm used
    print 'get_joint_number() : ', r.get_joint_number()

    #get_desired_joint_position() will return the desired positon
    #of the robot in terms of joint coordinates
    print 'get_desired_joint_position() : \n', r.get_desired_joint_position()

    #get_current_joint_position() will return the current positon
    #of the robot in terms of joint coordinate
    print 'get_current_joint_position() : \n', r.get_current_joint_position()
    
if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run getters()."""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        getters(sys.argv[1])
