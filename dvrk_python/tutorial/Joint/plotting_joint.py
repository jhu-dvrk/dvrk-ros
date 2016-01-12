"""In this example we will take a look at the how our robot will move in terms of a specific joint. Please note we can move a different distance than the one specified. For this example we are using Matplotlib. More information on the software can be found here at http://matplotlib.org. Lets take a look:"""

#we will need to import these classes from matplotlib in order to be able to plot
import pylab
from robot import*

#there are global variables that will record the position of the joints
current=[]
current = [0 for x in range(100)]
desired=[]
desired = [0 for x in range(100)]

def generate_current(args):
    """Here is where we generate the next current position we do this by getting the next current position and appending it to the current array."""
    global ro
    global joint
    current_joint = ro.get_current_joint_position()
    current.append(current_joint[joint])

def generate_desired(args):
    """Here is where we generate the next desired positon we do this by getting the next desired position and appending it to the desired array."""
    global ro
    global joint
    desired_joint = ro.get_desired_joint_position()
    desired.append(desired_joint[joint])

def plotter(robot_name):
    """Here is where we arrange the plotter to show only only a given amount of time and position at any given time we also use the plotter to plot the current position and the desired position"""
    global ro
    global joint
    CurrentXAxis=pylab.arange(len(current)-100,len(current),1)
    #here is where we add the next position to the array
    line1[0].set_data(CurrentXAxis,pylab.array(current[-100:]))
    line2[0].set_data(CurrentXAxis,pylab.array(desired[-100:]))
    #set the axis to show only the current joint positions
    ax.axis([CurrentXAxis.min(),CurrentXAxis.max(),-0.5,0.5])
    #update the graph
    manager.canvas.draw()


if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run plotting_cartesian(). We can change the joint being plotted by changing the joint value"""
    global ro
    global joint
    #change the value of joint will change what joint is being graphed
    joint = 1 #we are currently graphing joint 1
    #here will check if we are given the name of the robot arm or not, if we are we run plotter()"""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        ro = robot(sys.argv[1])
        
        #here is where we start creating the graph
        xAchse=pylab.arange(0,100,1)
        yAchse=pylab.array([0]*100)

        fig = pylab.figure(1)
        ax = fig.add_subplot(111)
        ax.grid(True)
        ax.set_title("Joint Degree vs. Time for Joint %d" %joint)
        ax.set_xlabel("Time")
        ax.set_ylabel("Degrees")
        ax.axis([0,100,-0.5,0.5])
        line1=ax.plot(xAchse,yAchse,'g')
        line2=ax.plot(xAchse,yAchse,'r')

        #here is where we start the timer
        manager = pylab.get_current_fig_manager()
        timer = fig.canvas.new_timer(interval=10)
        timer.add_callback(plotter, ())
        timer2 = fig.canvas.new_timer(interval=10)
        timer2.add_callback(generate_current, ())
        timer3 = fig.canvas.new_timer(interval=10)
        timer3.add_callback(generate_desired, ())
        timer.start()
        timer2.start()
        timer3.start()

        #keep showing the joint current and desired positions
        pylab.show()
