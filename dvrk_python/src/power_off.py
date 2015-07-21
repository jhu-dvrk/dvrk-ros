"""In example we will learn how to turn off the robot. We will show how the following methods work:
  * shutdown()

Lets take a look:"""

from robot import *

def power_off(robotName):
    """Here initialize the robot, in this demo we called this robot r, and shutdown the robot.

    :param robotName: the name of the robot used """
    r = robot(robotName)

    #shutdown() will turn off the robot
    r.shutdown()
   
if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run power_off()."""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        power_off(sys.argv[1])
