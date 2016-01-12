from robot import *
import math
import time

#trace out a polygon of side `sides` and length `length`
def polygon(robotName):
    r = robot(robotName)

    input_sides = raw_input('How many sides : ')
    sides = int(input_sides)
    input_length = raw_input('What is the length : ')
    length = float(input_length)

    #get the interior degree of the polygon, in terms of radians
    interior_degrees = 360/sides
    interior_radians = math.radians(interior_degrees)
    
    #find the distance of one point from the center
    interior_div2 = interior_radians/2
    distance_from_center = length/math.sin(interior_div2)
    
    #calcuate where the next position should be, based on the previous position
    current_angle = 0
    while(current_angle <= (2*math.pi)):
        #we made this program, based on a unit circle
        x_position = distance_from_center*math.cos(current_angle)
        y_position = distance_from_center*math.sin(current_angle)
        vec_1 = Vector(x_position,y_position, -0.15)
        r.move_cartesian_translation(vec_1,True)
        current_angle+=interior_radians

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        polygon(sys.argv[1])
