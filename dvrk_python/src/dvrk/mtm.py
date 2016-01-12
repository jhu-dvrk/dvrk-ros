from dvrk.arm import *

class mtm(arm):
    """Simple robot API wrapping around ROS messages
    """
    # initialize the robot
    def __init__(self, mtm_name, ros_namespace = '/dvrk/'):
        # first call base class constructor
        self._arm__init_arm(mtm_name, ros_namespace)

        # publishers
        self.lock_orientation_publisher = rospy.Publisher(self._arm__full_ros_namespace + '/lock_orientation',
                                                          Quaternion, latch=True, queue_size = 1)

        self.unlock_orientation_publisher = rospy.Publisher(self._arm__full_ros_namespace + '/unlock_orientation',
                                                            Empty, latch=True, queue_size = 1)

    def lock_orientation_as_is(self):
        "Lock orientation based on current orientation"
        current = self.get_desired_cartesian_position()
        self.lock_orientation(current.M)

    def lock_orientation(self, orientation):
        "Lock orientation"
        q = Quaternion()
        q.x, q.y, q.z, q.w = orientation.GetQuaternion()
        self.lock_orientation_publisher.publish(q);

    def unlock_orientation(self):
        "Lock orientation"
        e = Empty()
        self.unlock_orientation_publisher.publish(e);

