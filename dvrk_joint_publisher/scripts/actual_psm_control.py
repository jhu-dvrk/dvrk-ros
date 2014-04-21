#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

jnt_msg = JointState()

def jnt_pos_cb(msg):
    if len(msg.position) != 12:
        print "Error msg.position len was supposed to be 12 but is = ", len(msg.position)
        pass
    else:
        jnt_msg.position = []
        jnt_msg.position.append(msg.position[0])
        jnt_msg.position.append(msg.position[1])
        jnt_msg.position.append(msg.position[6]*1000)
        jnt_msg.position.append(msg.position[7])
        jnt_msg.position.append(msg.position[8])
        jnt_msg.position.append(msg.position[9])
        jnt_msg.position.append(msg.position[10])


def main():
    # initialize ROS node
    rospy.init_node('psm_joint_publisher')

    # create a psm publisher
    jnt_pub = rospy.Publisher('/dvrk_psm/set_position_joint', JointState)

    # create psm joint position subscriber
    jnt_pos_sub = rospy.Subscriber(
        'joint_states/joint_position_current',
        JointState,
        jnt_pos_cb)


    # initialize jnt_msg
    jnt_msg.name = ['one_outer_yaw_joint',
                    'one_outer_pitch_joint_1',
                    'one_outer_insertion_joint', 
                    'one_outer_roll_joint',
                    'one_outer_wrist_pitch_joint',
                    'one_outer_wrist_yaw_joint',
		    														'one_outer_wrist_open_angle_joint_1']

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

