"""In example we will learn how to turn on the robot. We will show how the following methods work:
 * home()

Lets take a look:"""

#we are using the robot api and therefore we will need to import this class
from robot import *

def power_on(robotName):
    """Here is where we initialize the robot, in this demo we called the robot r, and power the robot.

    :param robotName: the name of the robot used """
    r = robot(robotName)

    #`r.home()` will turn on and home the robot
    r.home()
    
if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run power_on() with the name of the robot."""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        power_on(sys.argv[1])
