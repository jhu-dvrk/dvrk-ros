#Give points given by the user go and pick up the objects at these points and move it to a common bucket.
from robot import *

def picking(robotName):
    r = robot(robotName)

    #ask the user for all pieces we have to pick up
    inputNum = raw_input('How many inputs do you have: ')
    #location where the user will drop off the object
    bucketIn = raw_input('Enter the bucket coordinates of the object as floats use a "," to seperate the coordinates: ')
    end_x, end_y, end_z = bucket.split(",")
    bucket = [end_x, end_y, end_z + 0.1]

    #ask the user for the objects to pick up go to that position
    #pick up the object
    for x in range (0, inputNum):
        start = raw_input('Enter the start coordinates of the object as floats use a "," to seperate the coordinates: ')
        start_x, start_y, start_z = start.split(",")

        r.move_cartesian_translation([start_x,start_y,start_z])
        time.sleep(1)
        pickup(r)
        time.sleep(1)
        r.move_cartesian_translation(bucket)
        r.open_gripper()
        time.sleep(1)

#picks up the object, we do this by moving to the location opening the gripper,
#translation downwards and close gripper
def pickup(robotName):
    r.open_gripper()
    time.sleep(1);
    r.delta_move_cartesian([0.0,0.0,-0.05])
    r.close_gripper()
    time.sleep(3);

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        picking(sys.argv[1])
