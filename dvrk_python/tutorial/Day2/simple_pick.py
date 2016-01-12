#Give 3 points given by the user go and pick up the objects at these points and move it to a common bucket.
from robot import *

def picking(robotName):
    r = robot(robotName)

    #what is the starting position
    start_input = raw_input('Enter the start coordinates of the object as floats use a "," to seperate the coordinates: ')
    start_x, start_y, start_z = start_input.split(",");
    origin_x = start_x;
    origin_y = start_y;
    
    r.move_cartesian_translation([start_x, start_y, start_z)

    #ask the user for all pieces we have to pick up
    inputNum = raw_input('How many inputs do you have: ')

    #location where the user will drop off the object
    output_x = raw_input('How far away in the x direction are the ending points : ')

    #ending position
    end_pos = [start_x, start_y, start_z + output_x];

    #how far away are the objects to be picked up located from one another
    output_y = raw_input('How far away in the y direction is one object from the previous one : ')
    

    #ask the user for the objects to pick up go to that position
    #pick up the object
    for x in range (0, inputNum - 1):
        pickup(r)
        time.sleep(1)
        start_y = start_y + output_y
        r.delta_move_cartesian_translation([start_x + output_x, start_y, start_z])
        r.open_gripper()
        time.sleep(1)
        r.delta_move_cartesian_translation([start_x, start_y, start_z])
    r.move_cartesian_translation(end_pos)

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
