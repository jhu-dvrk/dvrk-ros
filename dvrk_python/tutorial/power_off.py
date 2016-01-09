"""In example we will learn how to turn off the robot. We will show how the following methods work:
  * shutdown()

Lets take a look:"""

# we are using the robot api and therefore we will need to import this class
from dvrk_python.robot import *

# check if we are given the name of the robot arm or not, if we are we can create a robot and call shutdown()
if (len(sys.argv) != 2):
    print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'

else:
    robotName = sys.argv[1]
    # initialize the robot, in this demo we called the robot r, and power the robot.
    r = robot(robotName)

    # shutdown() will turn off the robot
    r.shutdown()
