"""In this example we will take a look at the how our robot will move in terms of cartesian position. Please note we can move a different distance than the one specified. For this example we are using Matplotlib. More information on the software can be found here at http://matplotlib.org. Lets take a look:"""

from robot import*

#please note you also have to import these methods in order to plot the movement
#import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import pylab as p

global current_x
global current_y
global current_z
global desired_x
global desired_y
global desired_z

current_x = []
current_y = []
current_z = []
desired_x = []
desired_y = []
desired_z = []

def update_data(arg):
    """This is where we update the data based on the current position and the desired position of the robot.

    :param arg: blank input"""
    global ro
    global ax
    global fig
    current_position = ro.get_current_cartesian_position()
    current_x.append(current_position.p.x())
    current_y.append(current_position.p.y())
    current_z.append(current_position.p.z())

    desired_position = ro.get_desired_cartesian_position()
    desired_x.append(desired_position.p.x())
    desired_y.append(desired_position.p.y())
    desired_z.append(desired_position.p.z())
    if (len(current_x) == 2):
        plt.plot(np.array(current_x), np.array(current_y), np.array(current_z), 'b')
        plt.draw
        plt.plot(np.array(desired_x), np.array(desired_y), np.array(desired_z), 'r')
        plt.draw()
    if (len(current_x) >= 2):
        current_x.pop(0)
        current_y.pop(0)
        current_z.pop(0)
        desired_x.pop(0)
        desired_y.pop(0)
        desired_z.pop(0)
    
def plotting_cartesian(robotName):
    """Here is where we initialize the robot, in this demo we called the robot r, and we move plot the current position in cartesian space.

    :param robotName: the name of the robot used """
    global ro
    global ax
    global fig
    ro = robot(robotName)

    #here we are declaring what type of graph we are using
    #in this case we will be using a 3D graph
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    #here is where we set the title of the graph
    ax.set_title("Cartesian Position over time")

    #here is where we set the limits for the axis of the graph
    ax.set_xlim3d(0,10)
    ax.set_ylim3d(0,10)
    ax.set_zlim3d(0,10)

    #here is where we set the x,y,z labels for the graph
    ax.set_xlabel('x - axis')
    ax.set_ylabel('y - axis')
    ax.set_zlabel('z - axis')

    #here is where we start the timer
    timer = fig.canvas.new_timer(interval=0)
    timer.add_callback(update_data, ())
    timer.start()

    plt.show(False)

if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run plotting_cartesian()."""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        plotting_cartesian(sys.argv[1])
