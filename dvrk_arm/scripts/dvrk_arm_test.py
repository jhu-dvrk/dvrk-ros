#!/usr/bin/env python

import rospy
from std_msgs.msg import String

robot_state = 'null'

def robot_state_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " -> current state is %s", data.data)
    global robot_state
    robot_state = data.data

if __name__ == '__main__':
    try:
        global robot_state

        # publishers
        set_robot_state = rospy.Publisher('/dvrk/ECM/set_robot_state', String, latch=True)

        # subscribers
        rospy.Subscriber("/dvrk/ECM/robot_state", String, robot_state_callback)


        # create node
        rospy.init_node('dvrk_arm_test', anonymous=True)
        rospy.loginfo(rospy.get_caller_id() + ' -> started dvrk_arm_test')

        rospy.loginfo(rospy.get_caller_id() + ' -> requesting homing')
        set_robot_state.publish('Home')

        counter = 60 # 60 seconds countdown
        while (counter > 0) and (robot_state != 'DVRK_READY'):
            counter = counter - 1
            rospy.loginfo(rospy.get_caller_id() + ' -> waiting for state to be DVRK_READY')
            rospy.sleep(1)

        if (robot_state != 'DVRK_READY'):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state DVRK_READY')
            rospy.signal_shutdown('failed to reach state DVRK_READY')

        rospy.sleep(20)

    except rospy.ROSInterruptException:
        pass
