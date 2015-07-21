from robot import *
import math

def polygon(robotName, sides, length):
    r = robot(robotName)

    interior_degrees = 360/sides
    interior_radians = math.radians(interior_degrees)
    
    interior_div2 = interior_radians/2
    distance_from_center = length/math.sin(interior_div2)
    
    current_angle = 0
    while(current_angle <= (2*math.pi)):
        x_position = distance_from_center*math.cos(current_angle)
        y_position = distance_from_center*math.sin(current_angle)
        vec_1 = Vector(x_position,y_position, -0.15)
        r.move_cartesian_translation(vec_1,True)
        current_angle+=interior_radians

if __name__ == '__main__':
    if (len(sys.argv) != 4):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        polygon(sys.argv[1], int(sys.argv[2]), float(sys.argv[3]))
