from robot import *
import math

def polygon(robotName):
    sides = 3
    length = 0.1
    r = robot(robotName)

    if type(sides) is int:
        r.move_cartesian_translation([0.0,0.0,0.15])
        print 'start',r.get_desired_cartesian_position()
        
        sum_interior = sides-2
        sum_interior_angle = sum_interior*180
        interior_degree = sum_interior_angle/sides
        interior_radian = interior_angle.radians(interior_degree)

        print 'in ', interior_radian
        #exterior_angle = 180 - interior_angle
        #print 'ex ', exterior_angle
        
        r.delta_move_cartesian_translation([length, 0.0, 0.15])
        
        i = 1
        while(i < sides):
            rot_1 = Rotation()
            rot_1.DoRotX()
            print 'step', i
            x_position = math.cos(exterior_angle) * length
            y_position = math.sin(exterior_angle) * length
            r.delta_move_cartesian_translation([x_position,y_position,0.15])
            print 'go', r.get_desired_cartesian_position()
            exterior_angle += exterior_angle
            i+=1

        print 'end', r.get_desired_cartesian_position()

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        polygon(sys.argv[1])
