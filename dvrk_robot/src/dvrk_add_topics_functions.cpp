/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2015-04-33

  (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <dvrk_utilities/dvrk_add_topics_functions.h>

void dvrk::add_topics_footpedal(mtsROSBridge & bridge,
                                const std::string & ros_namespace)
{
    bridge.AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        ("Clutch", "Button", ros_namespace + "/clutch");
    bridge.AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        ("Coag", "Button", ros_namespace + "/coag");
    bridge.AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        ("Camera", "Button", ros_namespace + "/camera");
    bridge.AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        ("Cam+", "Button", ros_namespace + "/camera_plus");
    bridge.AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        ("Cam-", "Button", ros_namespace + "/camera_minus");

}

void dvrk::connect_bridge_footpedal(mtsROSBridge & bridge,
                                    const std::string & io_component_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge.GetName(), "Clutch", io_component_name, "CLUTCH");
    componentManager->Connect(bridge.GetName(), "Coag", io_component_name, "COAG");
    componentManager->Connect(bridge.GetName(), "Camera", io_component_name, "CAMERA");
    componentManager->Connect(bridge.GetName(), "Cam+", io_component_name, "CAM+");
    componentManager->Connect(bridge.GetName(), "Cam-", io_component_name, "CAM-");
}

void dvrk::add_topics_arm(mtsROSBridge & bridge,
                          const std::string & ros_namespace,
                          const std::string & arm_component_name)
{
    // read
    bridge.AddPublisherFromCommandRead<prmPositionJointGet, sensor_msgs::JointState>
        (arm_component_name, "GetPositionJoint", ros_namespace + "/position_joint_current");
    bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (arm_component_name, "GetPositionJointDesired", ros_namespace + "/position_joint_desired");
    bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        (arm_component_name, "GetStateJoint", ros_namespace + "/state_joint_current");
    bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::Pose>
        (arm_component_name, "GetPositionCartesian", ros_namespace + "/position_cartesian_current");
    bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::Pose>
        (arm_component_name, "GetPositionCartesianDesired", ros_namespace + "/position_cartesian_desired");

    // write
    bridge.AddSubscriberToCommandWrite<std::string, std_msgs::String>
        (arm_component_name, "SetRobotControlState", ros_namespace + "/set_robot_state");
    bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
        (arm_component_name, "SetPositionJoint", ros_namespace + "/set_position_joint");
    bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
        (arm_component_name, "SetPositionGoalJoint", ros_namespace + "/set_position_goal_joint");
    bridge.AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::Pose>
        (arm_component_name, "SetPositionCartesian", ros_namespace + "/set_position_cartesian");
    bridge.AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::Pose>
        (arm_component_name, "SetPositionGoalCartesian", ros_namespace + "/set_position_goal_cartesian");

    // events
    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (arm_component_name, "Error", ros_namespace + "/error");
    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (arm_component_name, "Warning", ros_namespace + "/warning");
    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (arm_component_name, "Status", ros_namespace + "/status");

    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (arm_component_name, "RobotState", ros_namespace + "/robot_state");
    bridge.AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (arm_component_name, "GoalReached", ros_namespace + "/goal_reached");

    // messages
    bridge.AddLogFromEventWrite(arm_component_name + "-log", "Error", mtsROSEventWriteLog::ROS_LOG_ERROR);
    bridge.AddLogFromEventWrite(arm_component_name + "-log", "Warning", mtsROSEventWriteLog::ROS_LOG_WARN);
    bridge.AddLogFromEventWrite(arm_component_name + "-log", "Status", mtsROSEventWriteLog::ROS_LOG_INFO);
}

void dvrk::add_topics_mtm(mtsROSBridge & bridge,
                          const std::string & ros_namespace,
                          const std::string & mtm_component_name)
{
    // arm API
    dvrk::add_topics_arm(bridge, ros_namespace, mtm_component_name);

    // mtm specific API
    bridge.AddPublisherFromCommandRead<double, std_msgs::Float32>
        (mtm_component_name, "GetGripperPosition", ros_namespace + "/gripper_position_current");
    bridge.AddPublisherFromEventVoid
        (mtm_component_name, "GripperPinchEvent", ros_namespace + "/gripper_pinch_event");
    bridge.AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (mtm_component_name, "GripperClosedEvent", ros_namespace + "/gripper_closed_event");
}

void dvrk::connect_bridge_mtm(mtsROSBridge & bridge,
                              const std::string & mtm_component_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge.GetName(), mtm_component_name, mtm_component_name, "Robot");
    componentManager->Connect(bridge.GetName(), mtm_component_name + "-log", mtm_component_name, "Robot");
}

void dvrk::add_topics_psm(mtsROSBridge & bridge,
                          const std::string & ros_namespace,
                          const std::string & psm_component_name)
{
    // arm API
    dvrk::add_topics_arm(bridge, ros_namespace, psm_component_name);

    // psm specific API
    bridge.AddSubscriberToCommandWrite<double, std_msgs::Float32>
        (psm_component_name, "SetJawPosition", ros_namespace + "/set_jaw_position");

    // events
    bridge.AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        (psm_component_name, "ManipClutch", ros_namespace + "/manip_clutch");
    bridge.AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        (psm_component_name, "SUJClutch", ros_namespace + "/suj_clutch");
}

void dvrk::connect_bridge_psm(mtsROSBridge & bridge,
                              const std::string & psm_component_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge.GetName(), psm_component_name, psm_component_name, "Robot");
    componentManager->Connect(bridge.GetName(), psm_component_name + "-log", psm_component_name, "Robot");
}

void dvrk::add_topics_ecm(mtsROSBridge & bridge,
                          const std::string & ros_namespace,
                          const std::string & ecm_component_name)
{
    // arm API
    dvrk::add_topics_arm(bridge, ros_namespace, ecm_component_name);

    // ecm specific API

    // events
    bridge.AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        (ecm_component_name, "ManipClutch", ros_namespace + "/manip_clutch");
    bridge.AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        (ecm_component_name, "SUJClutch", ros_namespace + "/suj_clutch");
}

void dvrk::connect_bridge_ecm(mtsROSBridge & bridge,
                              const std::string & ecm_component_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge.GetName(), ecm_component_name, ecm_component_name, "Robot");
    componentManager->Connect(bridge.GetName(), ecm_component_name + "-log", ecm_component_name, "Robot");
}

void dvrk::add_topics_io(mtsROSBridge & bridge,
                         const std::string & ros_namespace,
                         const std::string & arm_name)
{
    bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (arm_name + "-io", "GetAnalogInputPosSI", ros_namespace + "/analog_input_pos_si");
    bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (arm_name + "-io", "GetPosition", ros_namespace + "/joint_position");

}

void dvrk::connect_bridge_io(mtsROSBridge & bridge,
                             const std::string & io_component_name,
                             const std::string & arm_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge.GetName(), arm_name + "-io", io_component_name, arm_name);
}
