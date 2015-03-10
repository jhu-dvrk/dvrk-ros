/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen
  Created on: 2013-07-14

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// Brief: da Vinci PSM kinematics

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include <cisstRobot/robManipulator.h>
#include <cisstVector.h>

#include <cisst_ros_bridge/mtsCISSTToROS.h>
#include <cisst_ros_bridge/mtsROSToCISST.h>

#include "dvrk_kinematics/psm_logic.h"

// set up joint state variables
static vctDoubleVec psm_joint_current;
static vctDoubleVec psm_joint_command;
static vctDoubleVec psm_joint_command_prev;
static vctFrm4x4 psm_pose_current;
static vctFrm4x4 psm_pose_command;
static double mtm_gripper;
static int control_mode;


// command psm pose from teleop
void psm_cmd_pose_cb(const geometry_msgs::Pose &msg)
{
    mtsROSToCISST(msg, psm_pose_command);
}

// mtm gripper open angle
void mtm_gripper_cb(const std_msgs::Float32 &msg)
{
    mtm_gripper = msg.data;
}

// psm joint feedback from RVIZ (joint_state_publisher)
void psm_joint_feedback_cb(const sensor_msgs::JointState &msg)
{
    if (control_mode != 2) {
        psm_joint_current[0] = msg.position[0];  // outer_yaw_joint
        psm_joint_current[1] = msg.position[1];  // outer_pitch_joint
        psm_joint_current[2] = msg.position[7];  // outer_insertion_joint
        psm_joint_current[3] = msg.position[8];  // outer_roll_joint
        psm_joint_current[4] = msg.position[9];  // outer_wrist_pitch_joint
        psm_joint_current[5] = msg.position[10];  // outer_wrist_yaw_joint
        //    psm_joint_current[6] = msg.position[11]; // outer_wrist_open_angle_joint
    }
}

void psm_mode_cb(const std_msgs::Int8 &msg)
{
    if (msg.data >= 0 && msg.data <=PSM::MODE_TELEOP) {
        control_mode = msg.data;
        std::cout << "control_mode = " << control_mode << std::endl;
    }

    // mode = TELEOP
    if (msg.data == PSM::MODE_TELEOP) {
        psm_joint_command.Assign(psm_joint_current);
        psm_pose_command.Assign(psm_pose_current);

        std::cout << "psm_pose_command = " << psm_pose_command << std::endl;
    }
}

int main(int argc, char** argv)
{
    // ros initialization
    ros::init(argc, argv, "dvrk_psm_kinematics");
    ros::NodeHandle nh, nh_private("~");
    ros::Rate rate(200);  // 200 hz rate

    // parameter
    std::string robot_name;
    nh_private.getParam("robot_name", robot_name);

    // subscriber
    ros::Subscriber sub_psm_cmd =
            nh.subscribe("/dvrk_psm/cartesian_pose_command", 1, psm_cmd_pose_cb);
    ros::Subscriber sub_mtm_gripper =
        nh.subscribe("/dvrk_mtm/gripper_position", 1, mtm_gripper_cb);

    ros::Subscriber sub_psm_fb =
            nh.subscribe("/dvrk_psm/joint_states", 1, psm_joint_feedback_cb);

    ros::Subscriber sub_mode =
            nh.subscribe("/dvrk_psm/control_mode", 1, psm_mode_cb);

    // publisher
    ros::Publisher pub_psm_joint_state_cmd =
            nh.advertise<sensor_msgs::JointState>("/dvrk_psm/joint_states_command", 1);
    ros::Publisher pub_psm_pose_current =
            nh.advertise<geometry_msgs::Pose>("/dvrk_psm/cartesian_pose_current", 1);
    ros::Publisher pub_psm_enable_slider =
            nh.advertise<sensor_msgs::JointState>("/dvrk_psm/joint_state_publisher/enable_slider", 100);    

    // --- cisst robManipulator ---
    std::string filename = ros::package::getPath("dvrk_robot");
    filename.append("/config/dvpsm.rob");
    robManipulator psm_manip;
    robManipulator::Errno result;
    result = psm_manip.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        ROS_ERROR("failed to load manipulator config file: %s", filename.c_str());
    } else {
        ROS_INFO("loaded psm manipulator");
    }

    // --- initialize joint current/command -----
    psm_joint_current.SetSize(6);  // 6 + 1 (open angle)
    psm_joint_current.SetAll(0.0);
    psm_joint_command.ForceAssign(psm_joint_current);

    sensor_msgs::JointState msg_js;
    msg_js.name.clear();
    msg_js.name.push_back(robot_name + "_outer_yaw_joint");
    msg_js.name.push_back(robot_name + "_outer_pitch_joint");
    msg_js.name.push_back(robot_name + "_outer_insertion_joint");
    msg_js.name.push_back(robot_name + "_outer_roll_joint");
    msg_js.name.push_back(robot_name + "_outer_wrist_pitch_joint");
    msg_js.name.push_back(robot_name + "_outer_wrist_yaw_joint");
    msg_js.name.push_back(robot_name + "_outer_wrist_open_angle_joint");

    geometry_msgs::Pose msg_pose;

    int count = 0;
    control_mode = PSM::MODE_RESET;  // start with reset_mode
    vctFrm4x4 frame6to7;
    frame6to7.Assign(0.0, -1.0, 0.0, 0.0,
                     0.0,  0.0, 1.0, 0.0102,
                     -1.0, 0.0, 0.0, 0.0,
                     0.0,  0.0, 0.0, 1.0);

    // used to compensate joint 4
    double j4_compensate = 0;

    // ------------ run() --------------------------
    while (ros::ok()) {

        // --------- Compute current pose & publish ----------
        // psm forward kinematics
        psm_pose_current = psm_manip.ForwardKinematics(psm_joint_current);
        mtsCISSTToROS(psm_pose_current, msg_pose);

        // publish current pose
        pub_psm_pose_current.publish(msg_pose);

        // ---------- Compute command psm joint positin -----------
        vctFrm4x4 pose6;

        // MODE_RESET: send HOME joint position
        // MODE_MANUAL: controled by JSP GUI, not sending anything to jsp
        // MODE_HOLD: disable JSP GUI, not sending anything to jsp
        // MODE_TELEOP: take command pose, send to jsp

        switch(control_mode)
        {
        case PSM::MODE_RESET:
            psm_joint_command.SetAll(0.0);
            psm_joint_command[2] = 0.10;
            j4_compensate = 0;
            psm_pose_command.Assign(psm_pose_current);
            break;
        case PSM::MODE_MANUAL:
            // do nothing for MANUAL
            // controlled using Slifer GUI
            j4_compensate = 0;
            psm_pose_command.Assign(psm_pose_current);

            std::fill(msg_js.position.begin(), msg_js.position.end(), 1);
            pub_psm_enable_slider.publish(msg_js);
            break;
        case PSM::MODE_HOLD:
            psm_pose_command.Assign(psm_pose_current);
            std::fill(msg_js.position.begin(), msg_js.position.end(), -1);
            pub_psm_enable_slider.publish(msg_js);
            break;
        case PSM::MODE_TELEOP:
            // psm_pose_command updated in callback!
            pose6 = psm_pose_command;
            psm_manip.InverseKinematics(psm_joint_command, pose6);

            // joint 4 is a special case
            if (psm_joint_command_prev[3] - psm_joint_command[3] > 5) {
                j4_compensate = 2 * cmnPI;
            } else if (psm_joint_command_prev[3] - psm_joint_command[3] < -5) {
                j4_compensate = - 2 * cmnPI;
            } else {
                j4_compensate = 0;
            }
            psm_joint_command[3] += j4_compensate;
            psm_joint_current.Assign(psm_joint_command);

            // disable slider
            std::fill(msg_js.position.begin(), msg_js.position.end(), -1);
            pub_psm_enable_slider.publish(msg_js);
            break;
        default:
            ROS_ERROR_STREAM("Should not come here !");
            break;
        }

        psm_joint_command_prev.ForceAssign(psm_joint_command);

        // -----  Assign command joint state and Publish --------
        // 6 + open angle (gripper)
        msg_js.position.resize(psm_joint_command.size() + 1);
        std::copy(psm_joint_command.begin(), psm_joint_command.end(), msg_js.position.begin());
        msg_js.position[6] = mtm_gripper;

        if (control_mode != PSM::MODE_MANUAL && control_mode != PSM::MODE_HOLD) {
            pub_psm_joint_state_cmd.publish(msg_js);
        }

        ros::spinOnce();
        rate.sleep();
        count++;
    }

    return 0;
}
