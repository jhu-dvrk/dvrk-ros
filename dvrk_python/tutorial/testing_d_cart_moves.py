from robot import *

def testing(robotName):
	r = robot(robotName)
	for i in range (0, 10):
		if i % 2 == 0:
			sign = 1.0
		else:
			sign = -1.0
		r.delta_move_cartesian([sign*(0.1),0.0,0.0])
		r.delta_move_cartesian([sign*(-0.1),0.0,0.0])

	for i in range (0, 10):
		if i % 2 == 0:
			sign = 1.0
		else:
			sign = -1.0
		r.delta_move_cartesian([0.0,sign*(0.1),0.0])
		r.delta_move_cartesian([0.0,sign*(-0.1),0.0])

	for i in range (0, 10):
		if i % 2 == 0:
			sign = 1.0
		else:
			sign = -1.0
		r.delta_move_cartesian([0.0,0.0,sign*(0.1)])
		r.delta_move_cartesian([0.0,0.0,sign*(-0.1)])

		

if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        testing(sys.argv[1])

