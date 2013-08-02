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

#include "irk_kinematics/mtm_logic.h"

// set up joint state variables
vctDoubleVec mtm_joint_current;
vctDoubleVec mtm_joint_command;
vctFrm4x4 mtm_pose_current;
int control_mode;

//const int MODE_RESET = 0;
//const int MODE_MANUAL = 1;
//const int MODE_TELEOP = 2;

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

void mtm_mode_cb(const std_msgs::Int8 &msg)
{
    if (msg.data >= 0 && msg.data <=MTM::MODE_TELEOP) {
        control_mode = msg.data;
        ROS_WARN("MTM switched to %d", msg.data);
    }
}

int main(int argc, char** argv)
{
    // ros initialization
    ros::init(argc, argv, "irk_mtm_logic");
    ros::NodeHandle nh;
    ros::Rate rate(200);  // 100 hz rate

    // subscriber
    ros::Subscriber sub_mtm_fb =
            nh.subscribe("/irk_mtm/joint_states", 1, mtm_joint_feedback_cb);
    ros::Subscriber sub_mode =
            nh.subscribe("/irk_mtm/control_mode", 1, mtm_mode_cb);

    // publisher
    ros::Publisher pub_mtm_joint_state =
            nh.advertise<sensor_msgs::JointState>("/irk_mtm/joint_states_command", 1);
    ros::Publisher pub_mtm_pose =
            nh.advertise<geometry_msgs::Pose>("/irk_mtm/cartesian_pose_current", 1);
    ros::Publisher pub_mtm_enable_slider =
            nh.advertise<sensor_msgs::JointState>("/irk_mtm/joint_state_publisher/enable_slider", 100);


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

    // initialize joint current/command/msg_js
    mtm_joint_current.SetSize(7);  // 7 joints
    mtm_joint_current.SetAll(0.0);

    mtm_joint_command.ForceAssign(mtm_joint_current);
    sensor_msgs::JointState msg_js;
    msg_js.name.clear();
    msg_js.name.push_back("right_outer_yaw_joint");
    msg_js.name.push_back("right_shoulder_pitch_joint");
    msg_js.name.push_back("right_elbow_pitch_joint");
    msg_js.name.push_back("right_wrist_platform_joint");
    msg_js.name.push_back("right_wrist_pitch_joint");
    msg_js.name.push_back("right_wrist_yaw_joint");
    msg_js.name.push_back("right_wrist_roll_joint");

    geometry_msgs::Pose msg_pose;

    int count = 0;
    // ------------ run() --------------------------
    while (ros::ok()) {

        // --------- Compute current pose & publish ----------
        // psm forward kinematics
        mtm_pose_current = mtm_manip.ForwardKinematics(mtm_joint_current);

        mtsCISSTToROS(mtm_pose_current, msg_pose);
//        if (count % 10 == 0) {
//            std::cerr << mtm_pose_current << std::endl << std::endl;
//        }

        // publish current pose
        pub_mtm_pose.publish(msg_pose);

        // MODE_RESET: send HOME joint position
        // MODE_MANUAL: controled by JSP GUI, not sending anything to jsp
        // MODE_HOLD: disable JSP GUI, not sending anything to jsp
        // MODE_CLUTCH: enable J1-3 slider, J4-7 follow
        // MODE_TELEOP: take command pose, send to jsp

        // ----- control mode -------
        switch (control_mode)
        {
        case MTM::MODE_RESET:
            mtm_joint_command.SetAll(0.0);
            mtm_joint_command[3] = cmnPI_2;
            mtm_joint_command[4] = cmnPI_2;
            break;
        case MTM::MODE_MANUAL:
            mtm_joint_command.ForceAssign(mtm_joint_current);
            std::fill(msg_js.position.begin(), msg_js.position.end(), 1);
            pub_mtm_enable_slider.publish(msg_js);
            break;
        case MTM::MODE_HOLD:
            mtm_joint_command.ForceAssign(mtm_joint_current);
            std::fill(msg_js.position.begin(), msg_js.position.end(), -1);
            pub_mtm_enable_slider.publish(msg_js);
            break;
        case MTM::MODE_CLUTCH:
            mtm_joint_command.ForceAssign(mtm_joint_current);
            std::fill(msg_js.position.begin(), msg_js.position.begin()+3, 1);
            std::fill(msg_js.position.begin()+3, msg_js.position.end(), -1);
            pub_mtm_enable_slider.publish(msg_js);
            break;
        case MTM::MODE_TELEOP:
            // teleop
            std::fill(msg_js.position.begin(), msg_js.position.end(), 1);
            pub_mtm_enable_slider.publish(msg_js);
            break;
        default:
            break;
        }

        // copy to msg
        msg_js.position.resize(mtm_joint_command.size());
        std::copy(mtm_joint_command.begin(), mtm_joint_command.end(), msg_js.position.begin());
        if (control_mode == MTM::MODE_RESET) {
            // publish cmd joint state
            pub_mtm_joint_state.publish(msg_js);
        }

        ros::spinOnce();
        rate.sleep();
        count++;
    }

    return 0;
}
