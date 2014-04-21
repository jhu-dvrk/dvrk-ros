#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

jnt_msg = JointState()

def jnt_pos_cb(msg):
    if len(msg.position) != 8:
        print "Error msg.position len was supposed to be 8 but is = ", len(msg.position)
        pass
    else:
        jnt_msg.position = []
        jnt_msg.position.append(msg.position[0])
        jnt_msg.position.append(msg.position[1])
        jnt_msg.position.append(msg.position[2])
        jnt_msg.position.append(msg.position[3])
        jnt_msg.position.append(msg.position[4])
        jnt_msg.position.append(msg.position[5])
        jnt_msg.position.append(msg.position[6])
        jnt_msg.position.append(msg.position[7])


def main():
    # initialize ROS node
    rospy.init_node('mtm_joint_publisher')

    # create a psm publisher
    jnt_pub = rospy.Publisher('/dvrk_mtm/set_position_joint', JointState)

    # create psm joint position subscriber
    jnt_pos_sub = rospy.Subscriber(
        'joint_states/joint_position_current',
        JointState,
        jnt_pos_cb)


    # initialize jnt_msg
    jnt_msg.name = ['right_outer_yaw_joint',
                    'right_shoulder_pitch_joint',
                    'right_elbow_pitch_joint', 
                    'right_wrist_platform_joint',
                    'right_wrist_pitch_joint',
                    'right_wrist_yaw_joint',
                    'right_wrist_roll_joint']

    # loop until ctrl-c
    rate = rospy.Rate(50);     # 50 hz
    while not rospy.is_shutdown():
        # update header
        jnt_msg.header.stamp = rospy.Time.now()

        # publish jointstate
        jnt_pub.publish(jnt_msg)

        # sleep
        rate.sleep()
        
# entry point
if __name__ == '__main__':
    main()

