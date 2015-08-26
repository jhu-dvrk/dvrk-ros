#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import JointState

jnt_msg = JointState()

def jnt_pos_cb(msg):
    global jnt_msg
    if len(msg.position) != 4:
        rospy.logerr("Error msg.position len was supposed to be 4 but is = " +
                     str(len(msg.position)))
        pass
    else:
        jnt_msg.position = []
        jnt_msg.position.append(msg.position[0])
        jnt_msg.position.append(msg.position[1])
        jnt_msg.position.append(msg.position[2])
        jnt_msg.position.append(msg.position[3])
        pass

def jnt_vel_cb(msg):
    global jnt_msg
    if len(msg.velocity) != 4:
        pass
    else:
        jnt_msg.velocity = []
        for i in range(0,len(msg.velocity)):
            jnt_msg.velocity.append(msg.velocity[i])
        pass

def main():
    global jnt_msg
    
    # initialize ROS node
    rospy.init_node('ecm_joint_publisher')

    # create a psm publisher
    jnt_pub = rospy.Publisher('joint_states_robot', JointState)

    # create psm joint position subscriber
    jnt_pos_sub = rospy.Subscriber(
        'joint_position_current',
        JointState,
        jnt_pos_cb)

    # create psm joint velocity subscriber
    jnt_vel_sub = rospy.Subscriber(
        'joint_velocity_current',
        JointState,
        jnt_vel_cb)

    # initialize jnt_msg
    jnt_msg.name = ['ecm_yaw_joint',
                    'ecm_pitch_joint',
                    'ecm_insertion_joint', 
                    'ecm_roll_joint']
    jnt_msg.position = [0, 0, 0, 0]
    

    # loop until ctrl-c
    rate = rospy.Rate(50);     # 50 hz
    while not rospy.is_shutdown():
        # update header
        jnt_msg.header.stamp = rospy.Time.now()

        if len(jnt_msg.name) != len(jnt_msg.position):
            rospy.logerr("ERROR Size mismatch")
            rospy.logerr("len.name = " + str(len(jnt_msg.name)))
            rospy.logerr("len.pos = " + str(len(jnt_msg.position)))

        # publish jointstate
        jnt_pub.publish(jnt_msg)

        # sleep
        rate.sleep()

# entry point
if __name__ == '__main__':
    main()

