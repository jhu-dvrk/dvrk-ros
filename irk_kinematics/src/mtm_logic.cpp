// Zihan Chen
// 2013-07-14
// Brief: da Vinci psm kinematics


#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include <cisstRobot/robManipulator.h>
#include <cisstVector.h>

#include <sawROS/mtsCISSTToROS.h>

// set up joint state variables
vctDoubleVec mtm_joint_current;
vctDoubleVec mtm_joint_command;
vctFrm4x4 mtm_pose_current;

void mtm_joint_feedback_cb(const sensor_msgs::JointStateConstPtr &msg)
{
    mtm_joint_current[0] = msg->position[0];   // outer_yaw_joint
    mtm_joint_current[1] = msg->position[1];   // shoulder_pitch_joint
    mtm_joint_current[2] = msg->position[3];   // elbow_pitch_joint
    mtm_joint_current[3] = msg->position[4];   // wrist_platform_joint
    mtm_joint_current[4] = msg->position[5];   // wrist_pitch_joint
    mtm_joint_current[5] = msg->position[6];   // wrist_yaw_joint
    mtm_joint_current[6] = msg->position[7];   // wrist_roll_joint
}


int main(int argc, char** argv)
{
    // ros initialization
    ros::init(argc, argv, "irk_mtm_logic");
    ros::NodeHandle nh;
    ros::Rate rate(100);  // 100 hz rate

    // subscriber
    ros::Subscriber sub_mtm_fb =
            nh.subscribe("/irk_mtm/joint_states", 1000, mtm_joint_feedback_cb);

    // publisher
    ros::Publisher pub_mtm_joint_state =
            nh.advertise<sensor_msgs::JointState>("/irk_mtm/joint_states", 1000);
    ros::Publisher pub_mtm_pose =
            nh.advertise<geometry_msgs::Pose>("/irk_mtm/cartesian_pose_current", 1000);


    // cisst robManipulator
    std::string filename = ros::package::getPath("irk_kinematics");
    filename.append("/config/dvmtm.rob");
    robManipulator mtm_manip;
    robManipulator::Errno result;
    result = mtm_manip.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        ROS_ERROR("failed to load manipulator config file: %s", filename.c_str());
    } else {
        ROS_INFO("loaded psm manipulator");
    }

    mtm_joint_current.SetSize(7);  // 7 joints
    mtm_joint_current.SetAll(0.0);

//    mtm_joint_command.ForceAssign(mtm_joint_current);

    geometry_msgs::Pose msg_pose;

    int count = 0;
    // ------------ run() --------------------------
    while (ros::ok()) {

        // --------- Compute current pose & publish ----------
        // psm forward kinematics
        mtm_pose_current = mtm_manip.ForwardKinematics(mtm_joint_current);

        mtsCISSTToROS(mtm_pose_current, msg_pose);
        if (count % 10 == 0) {
//            std::cerr << mtm_pose_current << std::endl << std::endl;
        }

        // publish current pose
        pub_mtm_pose.publish(msg_pose);

        // ---------- Compute command psm joint positin -----------

        // 2 sec, set robot to start position
        // after, in teleop mode
//        vctFrm4x4 pose6;
//        mtm_manip.InverseKinematics(mtm_joint_command, pose6);

        // publish command psm_joint_state
//        pub_mtm_joint_state.publish(msg_js);

        ros::spinOnce();

        rate.sleep();
        count++;
    }

    return 0;
}
