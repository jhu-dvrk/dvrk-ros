from dvrk.arm import *

class psm(arm):
    """Simple robot API wrapping around ROS messages
    """
    # initialize the robot
    def __init__(self, psm_name, ros_namespace = '/dvrk/'):
        # first call base class constructor
        self._arm__init_arm(psm_name, ros_namespace)
        
        # publishers
        self.set_jaw_position_publisher = rospy.Publisher(self._arm__full_ros_namespace
                                                          + '/set_jaw_position',
                                                          Float32, latch=True, queue_size = 1)

    def close_jaw(self):
        "Close the tool jaw"
        if (not self._arm__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        self.set_jaw_position_publisher.publish(-10.0 * math.pi / 180.0)

    def open_jaw(self):
        "Open the tool jaw"
        if (not self._arm__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        self.set_jaw_position_publisher.publish(80.0 * math.pi / 180.0)

    def set_jaw(self, set_jaw):
        "Set the jaw tool to set_jaw"
        if (not self._arm__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        if (set_jaw >= -10.0 * math.pi / 180.0 and set_jaw <= 80.0 * math.pi / 180.0):
            self.set_jaw_position_publisher.publish(set_jaw)
        else:
            print 'not a valid jaw position'
