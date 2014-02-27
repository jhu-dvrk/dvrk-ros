#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

jnt_msg = JointState()

def jnt_pos_cb(msg):
    if len(msg.position) != 8:
        print "len = ", len(msg.position)
        pass
    else:
        jnt_msg.position = []
        for i in range(0,7):
            jnt_msg.position.append(msg.position[i])
        pass

def jnt_vel_cb(msg):
    if len(msg.velocity) != 8:
        pass
    else:
        jnt_msg.velocity = []
        for i in range(0,7):
            jnt_msg.velocity.append(msg.velocity[i])
        pass

def main():
    # initialize ROS node
    rospy.init_node('mtm_joint_publisher')

    # create a mtm publisher
    jnt_pub = rospy.Publisher('/dvrk_mtm/joint_states_robot', JointState)

    # create mtm joint position subscriber
    jnt_pos_sub = rospy.Subscriber(
        'dvrk_mtm/joint_position_current',
        JointState,
        jnt_pos_cb)

    # create mtm joint velocity subscriber
    jnt_vel_sub = rospy.Subscriber(
        'dvrk_mtm/joint_velocity_current',
        JointState,
        jnt_vel_cb)

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

