from dvrk.arm import *

class psm(arm):
    """Simple robot API wrapping around ROS messages
    """
    # initialize the robot
    def __init__(self, psm_name, ros_namespace = '/dvrk/'):
        # first call base class constructor
        self._arm__init_arm(psm_name, ros_namespace)
        
        # publishers
        self.__set_jaw_position_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                      + '/set_jaw_position',
                                                      Float32, latch=True, queue_size = 1)
    def get_current_jaw_position(self):
        return self._arm__position_joint_current[6]

    def get_desired_jaw_position(self):
        return self._arm__position_joint_desired[6]

    def close_jaw(self):
        "Close the tool jaw"
        if (not self._arm__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        self.__set_jaw_position_pub.publish(-20.0 * math.pi / 180.0)

    def open_jaw(self):
        "Open the tool jaw"
        if (not self._arm__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        self.__set_jaw_position_pub.publish(80.0 * math.pi / 180.0)

    def move_jaw(self, set_jaw):
        "Set the jaw tool to set_jaw"
        if (not self._arm__dvrk_set_state('DVRK_POSITION_GOAL_CARTESIAN')):
            return False
        if (set_jaw >= -20.0 * math.pi / 180.0 and set_jaw <= 80.0 * math.pi / 180.0):
            self.__set_jaw_position_pub.publish(set_jaw)
        else:
            print 'not a valid jaw position'

    def insert_tool(self, depth):
        self.move_joint_some([depth], [2])

    def dinsert_tool(self, depth):
        self.dmove_joint_some([depth], [2])
