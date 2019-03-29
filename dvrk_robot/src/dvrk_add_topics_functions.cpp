/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2015-04-33

  (C) Copyright 2015-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <dvrk_utilities/dvrk_add_topics_functions.h>


void dvrk::add_topics_console(mtsROSBridge & bridge,
                              const std::string & ros_namespace,
                              const dvrk_topics_version::version CMN_UNUSED(version))
{
    bridge.AddSubscriberToCommandVoid
        ("Console", "PowerOff",
         ros_namespace + "/power_off");
    bridge.AddSubscriberToCommandVoid
        ("Console", "PowerOn",
         ros_namespace + "/power_on");
    bridge.AddSubscriberToCommandVoid
        ("Console", "Home",
         ros_namespace + "/home");
    bridge.AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        ("Console", "TeleopEnable",
         ros_namespace + "/teleop/enable");
    bridge.AddSubscriberToCommandWrite<std::string, std_msgs::String>
        ("Console", "CycleTeleopPSMByMTM",
         ros_namespace + "/teleop/cycle_teleop_psm_by_mtm");
    bridge.AddSubscriberToCommandWrite<prmKeyValue, diagnostic_msgs::KeyValue>
        ("Console", "SelectTeleopPSM",
         ros_namespace + "/teleop/select_teleop_psm");
    bridge.AddSubscriberToCommandWrite<double, std_msgs::Float32>
        ("Console", "SetScale",
         ros_namespace + "/teleop/set_scale");
    bridge.AddPublisherFromEventWrite<double, std_msgs::Float32>
        ("Console", "Scale",
         ros_namespace + "/teleop/scale");
    bridge.AddPublisherFromEventWrite<prmKeyValue, diagnostic_msgs::KeyValue>
        ("Console", "TeleopPSMSelected",
         ros_namespace + "/teleop/teleop_psm_selected");
    bridge.AddPublisherFromEventWrite<prmKeyValue, diagnostic_msgs::KeyValue>
        ("Console", "TeleopPSMUnselected",
         ros_namespace + "/teleop/teleop_psm_unselected");
}

void dvrk::connect_bridge_console(const std::string & bridge_name,
                                  const std::string & console_component_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, "Console",
                              console_component_name, "Main");
}

void dvrk::add_topics_footpedals(mtsROSBridge & bridge,
                                 const std::string & ros_namespace,
                                 const dvrk_topics_version::version version)
{
    switch (version) {
    case dvrk_topics_version::v1_3_0:
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
        break;
    default:
        bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            ("Clutch", "Button", ros_namespace + "/clutch");
        bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            ("Coag", "Button", ros_namespace + "/coag");
        bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            ("Camera", "Button", ros_namespace + "/camera");
        bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            ("Cam+", "Button", ros_namespace + "/camera_plus");
        bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            ("Cam-", "Button", ros_namespace + "/camera_minus");
        break;
    }
}

void dvrk::connect_bridge_footpedals(const std::string & bridge_name,
                                     const std::string & io_component_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, "Clutch",
                              io_component_name, "CLUTCH");
    componentManager->Connect(bridge_name, "Coag",
                              io_component_name, "COAG");
    componentManager->Connect(bridge_name, "Camera",
                              io_component_name, "CAMERA");
    componentManager->Connect(bridge_name, "Cam+",
                              io_component_name, "CAM+");
    componentManager->Connect(bridge_name, "Cam-",
                              io_component_name, "CAM-");
}

void dvrk::add_topics_arm(mtsROSBridge & bridge,
                          const std::string & ros_namespace,
                          const std::string & arm_component_name,
                          const dvrk_topics_version::version version)
{
    // read
    switch (version) {
    case dvrk_topics_version::v1_3_0:
        bridge.AddPublisherFromCommandRead<prmPositionJointGet, sensor_msgs::JointState>
            (arm_component_name, "GetPositionJoint",
             ros_namespace + "/position_joint_current");
        bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
            (arm_component_name, "GetPositionJointDesired",
             ros_namespace + "/position_joint_desired");
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (arm_component_name, "GetStateJoint",
             ros_namespace + "/state_joint_current");
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (arm_component_name, "GetStateJointDesired",
             ros_namespace + "/state_joint_desired");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::Pose>
            (arm_component_name, "GetPositionCartesianLocal",
             ros_namespace + "/position_cartesian_local_current");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::Pose>
            (arm_component_name, "GetPositionCartesianLocalDesired",
             ros_namespace + "/position_cartesian_local_desired");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::Pose>
            (arm_component_name, "GetPositionCartesian",
             ros_namespace + "/position_cartesian_current");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::Pose>
            (arm_component_name, "GetPositionCartesianDesired",
             ros_namespace + "/position_cartesian_desired");
        break;
    case dvrk_topics_version::crtk_alpha:
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (arm_component_name, "GetStateJoint",
             ros_namespace + "/measured_js");
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (arm_component_name, "GetStateJointDesired",
             ros_namespace + "/setpoint_js");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
            (arm_component_name, "GetPositionCartesianLocal",
             ros_namespace + "/local/measured_cp");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
            (arm_component_name, "GetPositionCartesianLocalDesired",
             ros_namespace + "/local/setpoint_cp");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
            (arm_component_name, "GetPositionCartesian",
             ros_namespace + "/measured_cp");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
            (arm_component_name, "GetPositionCartesianDesired",
             ros_namespace + "/setpoint_cp");
        bridge.AddPublisherFromCommandRead<prmVelocityCartesianGet, geometry_msgs::TwistStamped>
            (arm_component_name, "GetVelocityCartesian",
             ros_namespace + "/measured_cv");
        bridge.AddPublisherFromCommandRead<prmForceCartesianGet, geometry_msgs::WrenchStamped>
            (arm_component_name, "GetWrenchBody",
             ros_namespace + "/body/measured_cf");
        bridge.AddPublisherFromCommandRead<vctDoubleMat, std_msgs::Float64MultiArray>
            (arm_component_name, "GetJacobianBody",
             ros_namespace + "/body/jacobian");
        bridge.AddPublisherFromCommandRead<vctDoubleMat, std_msgs::Float64MultiArray>
            (arm_component_name, "GetJacobianSpatial",
             ros_namespace + "/spatial/jacobian");
        break;
    default:
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (arm_component_name, "GetStateJoint",
             ros_namespace + "/state_joint_current");
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (arm_component_name, "GetStateJointDesired",
             ros_namespace + "/state_joint_desired");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (arm_component_name, "GetPositionCartesianLocal",
             ros_namespace + "/position_cartesian_local_current");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (arm_component_name, "GetPositionCartesianLocalDesired",
             ros_namespace + "/position_cartesian_local_desired");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (arm_component_name, "GetPositionCartesian",
             ros_namespace + "/position_cartesian_current");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (arm_component_name, "GetPositionCartesianDesired",
             ros_namespace + "/position_cartesian_desired");
        bridge.AddPublisherFromCommandRead<prmVelocityCartesianGet, geometry_msgs::TwistStamped>
            (arm_component_name, "GetVelocityCartesian",
             ros_namespace + "/twist_body_current");
        bridge.AddPublisherFromCommandRead<prmForceCartesianGet, geometry_msgs::WrenchStamped>
            (arm_component_name, "GetWrenchBody",
             ros_namespace + "/wrench_body_current");
        bridge.AddPublisherFromCommandRead<vctDoubleMat, std_msgs::Float64MultiArray>
            (arm_component_name, "GetJacobianBody",
             ros_namespace + "/jacobian_body");
        bridge.AddPublisherFromCommandRead<vctDoubleMat, std_msgs::Float64MultiArray>
            (arm_component_name, "GetJacobianSpatial",
             ros_namespace + "/jacobian_spatial");
        break;
    }

    // write
    bridge.AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::Pose>
        (arm_component_name, "SetBaseFrame",
         ros_namespace + "/set_base_frame");
    bridge.AddSubscriberToCommandWrite<std::string, std_msgs::String>
        (arm_component_name, "SetDesiredState",
         ros_namespace + "/set_desired_state");
    bridge.AddSubscriberToCommandWrite<double, std_msgs::Float64>
        (arm_component_name, "SetJointVelocityRatio",
         ros_namespace + "/set_joint_velocity_ratio");
    bridge.AddSubscriberToCommandWrite<double, std_msgs::Float64>
        (arm_component_name, "SetJointAccelerationRatio",
         ros_namespace + "/set_joint_acceleration_ratio");

    switch (version) {
    case dvrk_topics_version::crtk_alpha:
        bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
            (arm_component_name, "SetPositionJoint",
             ros_namespace + "/servo_jp");
        bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
            (arm_component_name, "SetPositionGoalJoint",
             ros_namespace + "/move_jp");
        bridge.AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::TransformStamped>
            (arm_component_name, "SetPositionCartesian",
             ros_namespace + "/servo_cp");
        bridge.AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::TransformStamped>
            (arm_component_name, "SetPositionGoalCartesian",
             ros_namespace + "/move_cp");
        bridge.AddSubscriberToCommandWrite<prmForceTorqueJointSet, sensor_msgs::JointState>
            (arm_component_name, "SetEffortJoint",
             ros_namespace + "/servo_jf");
        bridge.AddSubscriberToCommandWrite<prmForceCartesianSet, geometry_msgs::WrenchStamped>
            (arm_component_name, "SetWrenchBody",
             ros_namespace + "/body/servo_cf");
        bridge.AddSubscriberToCommandWrite<prmForceCartesianSet, geometry_msgs::WrenchStamped>
            (arm_component_name, "SetWrenchSpatial",
             ros_namespace + "/spatial/servo_cf");
        break;
    default:
        bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
            (arm_component_name, "SetPositionJoint",
             ros_namespace + "/set_position_joint");
        bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
            (arm_component_name, "SetPositionGoalJoint",
             ros_namespace + "/set_position_goal_joint");
        bridge.AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::Pose>
            (arm_component_name, "SetPositionCartesian",
             ros_namespace + "/set_position_cartesian");
        bridge.AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::Pose>
            (arm_component_name, "SetPositionGoalCartesian",
             ros_namespace + "/set_position_goal_cartesian");
        bridge.AddSubscriberToCommandWrite<prmForceTorqueJointSet, sensor_msgs::JointState>
            (arm_component_name, "SetEffortJoint",
             ros_namespace + "/set_effort_joint");
        bridge.AddSubscriberToCommandWrite<prmForceCartesianSet, geometry_msgs::Wrench>
            (arm_component_name, "SetWrenchBody",
             ros_namespace + "/set_wrench_body");
        bridge.AddSubscriberToCommandWrite<prmForceCartesianSet, geometry_msgs::Wrench>
            (arm_component_name, "SetWrenchSpatial",
             ros_namespace + "/set_wrench_spatial");
        break;
    }

    bridge.AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (arm_component_name, "SetWrenchBodyOrientationAbsolute",
         ros_namespace + "/set_wrench_body_orientation_absolute");
    bridge.AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (arm_component_name, "SetGravityCompensation",
         ros_namespace + "/set_gravity_compensation");
    bridge.AddSubscriberToCommandWrite<prmCartesianImpedanceGains, cisst_msgs::prmCartesianImpedanceGains>
        (arm_component_name, "SetCartesianImpedanceGains",
         ros_namespace + "/set_cartesian_impedance_gains");


    // events
    bridge.AddPublisherFromEventWrite<mtsMessage, std_msgs::String>
        (arm_component_name, "Error",
         ros_namespace + "/error");
    bridge.AddPublisherFromEventWrite<mtsMessage, std_msgs::String>
        (arm_component_name, "Warning",
         ros_namespace + "/warning");
    bridge.AddPublisherFromEventWrite<mtsMessage, std_msgs::String>
        (arm_component_name, "Status",
         ros_namespace + "/status");

    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (arm_component_name, "CurrentState",
         ros_namespace + "/current_state");
    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (arm_component_name, "DesiredState",
         ros_namespace + "/desired_state");
    bridge.AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (arm_component_name, "GoalReached",
         ros_namespace + "/goal_reached");
    bridge.AddPublisherFromEventWrite<double, std_msgs::Float64>
        (arm_component_name, "JointVelocityRatio",
         ros_namespace + "/joint_velocity_ratio");
    bridge.AddPublisherFromEventWrite<double, std_msgs::Float64>
        (arm_component_name, "JointAccelerationRatio",
         ros_namespace + "/joint_acceleration_ratio");

    // messages
    bridge.AddLogFromEventWrite(arm_component_name + "-log", "Error",
                                mtsROSEventWriteLog::ROS_LOG_ERROR);
    bridge.AddLogFromEventWrite(arm_component_name + "-log", "Warning",
                                mtsROSEventWriteLog::ROS_LOG_WARN);
    bridge.AddLogFromEventWrite(arm_component_name + "-log", "Status",
                                mtsROSEventWriteLog::ROS_LOG_INFO);
}

void dvrk::add_topics_mtm(mtsROSBridge & bridge,
                          const std::string & ros_namespace,
                          const std::string & mtm_component_name,
                          const dvrk_topics_version::version version)
{
    // arm API
    dvrk::add_topics_arm(bridge, ros_namespace,
                         mtm_component_name, version);

    // mtm specific API
    bridge.AddSubscriberToCommandWrite<vctMatRot3, geometry_msgs::Quaternion>
        (mtm_component_name, "LockOrientation",
         ros_namespace + "/lock_orientation");
    bridge.AddSubscriberToCommandVoid
        (mtm_component_name, "UnlockOrientation",
         ros_namespace + "/unlock_orientation");
    bridge.AddPublisherFromEventVoid
        (mtm_component_name, "GripperPinchEvent",
         ros_namespace + "/gripper_pinch_event");
    bridge.AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (mtm_component_name, "GripperClosedEvent",
         ros_namespace + "/gripper_closed_event");

    switch (version) {
    case dvrk_topics_version::crtk_alpha:
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (mtm_component_name, "GetStateGripper",
             ros_namespace + "/gripper/measured_js");
        break;
    default:
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (mtm_component_name, "GetStateGripper",
             ros_namespace + "/state_gripper_current");
        break;
    }
}

void dvrk::add_topics_mtm_generic(mtsROSBridge & bridge,
                                  const std::string & ros_namespace,
                                  const std::string & arm_component_name,
                                  const dvrk_topics_version::version CMN_UNUSED(version))
{
    // read
    bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
        (arm_component_name, "GetPositionCartesian",
         ros_namespace + "/position_cartesian_current");
    bridge.AddPublisherFromCommandRead<prmVelocityCartesianGet, geometry_msgs::TwistStamped>
        (arm_component_name, "GetVelocityCartesian",
         ros_namespace + "/twist_body_current");
    bridge.AddPublisherFromCommandRead<prmForceCartesianGet, geometry_msgs::WrenchStamped>
        (arm_component_name, "GetWrenchBody",
         ros_namespace + "/wrench_body_current");
    bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        (arm_component_name, "GetStateGripper",
         ros_namespace + "/state_gripper_current");

    // write
    bridge.AddSubscriberToCommandWrite<std::string, std_msgs::String>
        (arm_component_name, "SetDesiredState",
         ros_namespace + "/set_robot_state");
    bridge.AddSubscriberToCommandWrite<prmForceCartesianSet, geometry_msgs::Wrench>
        (arm_component_name, "SetWrenchBody",
         ros_namespace + "/set_wrench_body");
    bridge.AddSubscriberToCommandWrite<prmForceTorqueJointSet, sensor_msgs::JointState>
        (arm_component_name, "SetEffortGripper",
         ros_namespace + "/set_effort_gripper");
    bridge.AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (arm_component_name, "SetGravityCompensation",
         ros_namespace + "/set_gravity_compensation");

    // events
    bridge.AddPublisherFromEventWrite<mtsMessage, std_msgs::String>
        (arm_component_name, "Error", ros_namespace + "/error");
    bridge.AddPublisherFromEventWrite<mtsMessage, std_msgs::String>
        (arm_component_name, "Warning", ros_namespace + "/warning");
    bridge.AddPublisherFromEventWrite<mtsMessage, std_msgs::String>
        (arm_component_name, "Status", ros_namespace + "/status");

    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (arm_component_name, "CurrentState", ros_namespace + "/current_state");
    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (arm_component_name, "DesiredState", ros_namespace + "/desired_state");

    // messages
    bridge.AddLogFromEventWrite(arm_component_name + "-log", "Error",
                                mtsROSEventWriteLog::ROS_LOG_ERROR);
    bridge.AddLogFromEventWrite(arm_component_name + "-log", "Warning",
                                mtsROSEventWriteLog::ROS_LOG_WARN);
    bridge.AddLogFromEventWrite(arm_component_name + "-log", "Status",
                                mtsROSEventWriteLog::ROS_LOG_INFO);
}

void dvrk::connect_bridge_mtm(const std::string & bridge_name,
                              const std::string & arm_name,
                              const std::string & mtm_component_name,
                              const std::string & mtm_interface_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, arm_name,
                              mtm_component_name, mtm_interface_name);
    componentManager->Connect(bridge_name, arm_name + "-log",
                              mtm_component_name, mtm_interface_name);
}

void dvrk::add_topics_psm(mtsROSBridge & bridge,
                          const std::string & ros_namespace,
                          const std::string & psm_component_name,
                          const dvrk_topics_version::version version)
{
    // arm API
    dvrk::add_topics_arm(bridge, ros_namespace,
                         psm_component_name, version);

    // psm specific API
    switch (version) {
    case dvrk_topics_version::crtk_alpha:
        bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
            (psm_component_name, "SetPositionJaw",
             ros_namespace + "/jaw/servo_jp");
        bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
            (psm_component_name, "SetPositionGoalJaw",
             ros_namespace + "/jaw/move_jp");
        bridge.AddSubscriberToCommandWrite<prmForceTorqueJointSet, sensor_msgs::JointState>
            (psm_component_name, "SetEffortJaw",
             ros_namespace + "/jaw/servo_jf");
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (psm_component_name, "GetStateJawDesired",
             ros_namespace + "/jaw/setpoint_js");
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (psm_component_name, "GetStateJaw",
             ros_namespace + "/jaw/measured_js");
        break;
    default:
        bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
            (psm_component_name, "SetPositionJaw",
             ros_namespace + "/set_position_jaw");
        bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
            (psm_component_name, "SetPositionGoalJaw",
             ros_namespace + "/set_position_goal_jaw");
        bridge.AddSubscriberToCommandWrite<prmForceTorqueJointSet, sensor_msgs::JointState>
            (psm_component_name, "SetEffortJaw",
             ros_namespace + "/set_effort_jaw");
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (psm_component_name, "GetStateJawDesired",
             ros_namespace + "/state_jaw_desired");
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (psm_component_name, "GetStateJaw",
             ros_namespace + "/state_jaw_current");
        break;
    }

    bridge.AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (psm_component_name, "SetAdapterPresent",
         ros_namespace + "/set_adapter_present");

    bridge.AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (psm_component_name, "SetToolPresent",
         ros_namespace + "/set_tool_present");

    // events
    bridge.AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        (psm_component_name, "ManipClutch",
         ros_namespace + "/manip_clutch");
}

void dvrk::connect_bridge_psm(const std::string & bridge_name,
                              const std::string & arm_name,
                              const std::string & psm_component_name,
                              const std::string & psm_interface_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, arm_name,
                              psm_component_name, psm_interface_name);
    componentManager->Connect(bridge_name, arm_name + "-log",
                              psm_component_name, psm_interface_name);
}

void dvrk::add_topics_psm_io(mtsROSBridge & bridge,
                             const std::string & ros_namespace,
                             const std::string & arm_name,
                             const dvrk_topics_version::version CMN_UNUSED(version))
{
    bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        (arm_name + "-ManipClutch", "Button", ros_namespace + "/io/manip_clutch");
    bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        (arm_name + "-SUJClutch", "Button", ros_namespace + "/io/suj_clutch");
    bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        (arm_name + "-Adapter", "Button", ros_namespace + "/io/adapter");
    bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        (arm_name + "-Tool", "Button", ros_namespace + "/io/tool");
}

void dvrk::connect_bridge_psm_io(const std::string & bridge_name,
                                 const std::string & arm_name,
                                 const std::string & io_component_name)
{
    std::string interfaceName;
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    interfaceName = arm_name + "-ManipClutch";
    componentManager->Connect(bridge_name, interfaceName,
                              io_component_name, interfaceName);
    interfaceName = arm_name + "-SUJClutch";
    componentManager->Connect(bridge_name, interfaceName,
                              io_component_name, interfaceName);
    interfaceName = arm_name + "-Adapter";
    componentManager->Connect(bridge_name, interfaceName,
                              io_component_name, interfaceName);
    interfaceName = arm_name + "-Tool";
    componentManager->Connect(bridge_name, interfaceName,
                              io_component_name, interfaceName);
}

void dvrk::add_topics_ecm(mtsROSBridge & bridge,
                          const std::string & ros_namespace,
                          const std::string & ecm_component_name,
                          const dvrk_topics_version::version version)
{
    // arm API
    dvrk::add_topics_arm(bridge, ros_namespace,
                         ecm_component_name, version);

    // ecm specific API

    // events
    switch (version) {
    case dvrk_topics_version::v1_3_0:
        bridge.AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
            (ecm_component_name, "ManipClutch", ros_namespace + "/manip_clutch");
        break;
    default:
        bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            (ecm_component_name, "ManipClutch", ros_namespace + "/manip_clutch");
        break;
    }
}

void dvrk::connect_bridge_ecm(const std::string & bridge_name,
                              const std::string & arm_name,
                              const std::string & ecm_component_name,
                              const std::string & ecm_interface_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, arm_name,
                              ecm_component_name, ecm_interface_name);
    componentManager->Connect(bridge_name, arm_name + "-log",
                              ecm_component_name, ecm_interface_name);
}

void dvrk::add_topics_ecm_io(mtsROSBridge & bridge,
                             const std::string & ros_namespace,
                             const std::string & arm_name,
                             const dvrk_topics_version::version CMN_UNUSED(version))
{
    bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        (arm_name + "-ManipClutch", "Button", ros_namespace + "/io/manip_clutch");
    bridge.AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        (arm_name + "-SUJClutch", "Button", ros_namespace + "/io/suj_clutch");
}

void dvrk::connect_bridge_ecm_io(const std::string & bridge_name,
                                 const std::string & arm_name,
                                 const std::string & io_component_name)
{
    std::string interfaceName;
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    interfaceName = arm_name + "-ManipClutch";
    componentManager->Connect(bridge_name, interfaceName,
                              io_component_name, interfaceName);
    interfaceName = arm_name + "-SUJClutch";
    componentManager->Connect(bridge_name, interfaceName,
                              io_component_name, interfaceName);
}

void dvrk::add_topics_teleop(mtsROSBridge & bridge,
                             const std::string & ros_namespace,
                             const std::string & teleop_component_name,
                             const dvrk_topics_version::version version)
{
    // messages
    bridge.AddLogFromEventWrite(teleop_component_name + "-log", "Error",
                                mtsROSEventWriteLog::ROS_LOG_ERROR);
    bridge.AddLogFromEventWrite(teleop_component_name + "-log", "Warning",
                                mtsROSEventWriteLog::ROS_LOG_WARN);
    bridge.AddLogFromEventWrite(teleop_component_name + "-log", "Status",
                                mtsROSEventWriteLog::ROS_LOG_INFO);

    // events
    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (teleop_component_name, "DesiredState", ros_namespace + "/desired_state");
    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (teleop_component_name, "CurrentState", ros_namespace + "/current_state");
    switch (version) {
    case dvrk_topics_version::v1_3_0:
        bridge.AddPublisherFromEventWrite<bool, std_msgs::Bool>
            (teleop_component_name, "RotationLocked", ros_namespace + "/rotation_locked");
        bridge.AddPublisherFromEventWrite<bool, std_msgs::Bool>
            (teleop_component_name, "TranslationLocked", ros_namespace + "/translation_locked");
        break;
    default:
        bridge.AddPublisherFromEventWrite<bool, sensor_msgs::Joy>
            (teleop_component_name, "RotationLocked",
             ros_namespace + "/rotation_locked");
        bridge.AddPublisherFromEventWrite<bool, sensor_msgs::Joy>
            (teleop_component_name, "TranslationLocked",
             ros_namespace + "/translation_locked");
        break;
    }
    bridge.AddPublisherFromEventWrite<double, std_msgs::Float32>
        (teleop_component_name, "Scale", ros_namespace + "/scale");
    bridge.AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (teleop_component_name, "Following", ros_namespace + "/following");

    // commands
    bridge.AddSubscriberToCommandWrite<std::string, std_msgs::String>
        (teleop_component_name, "SetDesiredState",
         ros_namespace + "/set_desired_state");
    bridge.AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (teleop_component_name, "LockTranslation",
         ros_namespace + "/lock_translation");
    bridge.AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (teleop_component_name, "LockRotation",
         ros_namespace + "/lock_rotation");
    bridge.AddSubscriberToCommandWrite<double, std_msgs::Float32>
        (teleop_component_name, "SetScale",
         ros_namespace + "/set_scale");
    bridge.AddSubscriberToCommandWrite<vctMatRot3, geometry_msgs::Quaternion>
        (teleop_component_name, "SetRegistrationRotation",
         ros_namespace + "/set_registration_rotation");
}

void dvrk::connect_bridge_teleop(const std::string & bridge_name,
                                 const std::string & teleop_component_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, teleop_component_name,
                              teleop_component_name, "Setting");
    componentManager->Connect(bridge_name, teleop_component_name + "-log",
                              teleop_component_name, "Setting");
}

void dvrk::add_topics_suj(mtsROSBridge & bridge,
                          const std::string & ros_namespace,
                          const std::string & arm_name,
                          const dvrk_topics_version::version version)
{
    // events
    switch (version) {
    case dvrk_topics_version::v1_3_0:
        bridge.AddPublisherFromEventWrite<prmPositionCartesianGet, geometry_msgs::Pose>
            (arm_name + "-suj", "PositionCartesian",
             ros_namespace + "/position_cartesian_current");
        bridge.AddPublisherFromEventWrite<prmPositionCartesianGet, geometry_msgs::Pose>
            (arm_name + "-suj", "PositionCartesianLocal",
             ros_namespace + "/position_cartesian_local_current");
        break;
    case dvrk_topics_version::crtk_alpha:
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (arm_name + "-suj", "GetStateJoint",
             ros_namespace + "/measured_js");
        bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
            (arm_name + "-suj", "SetPositionJoint",
             ros_namespace + "/servo_jp");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
            (arm_name + "-suj", "PositionCartesianLocal",
             ros_namespace + "/local/measured_cp");
        bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
            (arm_name + "-suj", "PositionCartesian",
             ros_namespace + "/measured_cp");
        break;
    default:
        bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
            (arm_name + "-suj", "GetStateJoint",
             ros_namespace + "/state_joint_current");
        bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
            (arm_name + "-suj", "SetPositionJoint",
             ros_namespace + "/set_position_joint");
        bridge.AddPublisherFromEventWrite<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (arm_name + "-suj", "PositionCartesian",
             ros_namespace + "/position_cartesian_current");
        bridge.AddPublisherFromEventWrite<prmPositionCartesianGet, geometry_msgs::PoseStamped>
            (arm_name + "-suj", "PositionCartesianLocal",
             ros_namespace + "/position_cartesian_local_current");
        break;
    }

    // messages
    bridge.AddLogFromEventWrite(arm_name + "-suj-log", "Error",
                                mtsROSEventWriteLog::ROS_LOG_ERROR);
    bridge.AddLogFromEventWrite(arm_name + "-suj-log", "Warning",
                                mtsROSEventWriteLog::ROS_LOG_WARN);
    bridge.AddLogFromEventWrite(arm_name + "-suj-log", "Status",
                                mtsROSEventWriteLog::ROS_LOG_INFO);
}

void dvrk::connect_bridge_suj(const std::string & bridge_name,
                              const std::string & suj_component_name,
                              const std::string & arm_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, arm_name + "-suj",
                              suj_component_name, arm_name);
    componentManager->Connect(bridge_name, arm_name + "-suj-log",
                              suj_component_name, arm_name);
}

void dvrk::add_topics_io(mtsROSBridge & bridge,
                         const std::string & ros_namespace,
                         const dvrk_topics_version::version CMN_UNUSED(version))
{
    bridge.AddPublisherFromCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        ("io", "GetPeriodStatistics",
         ros_namespace + "/period_statistics");
    bridge.AddPublisherFromCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        ("io", "GetPeriodStatisticsRead",
         ros_namespace + "/period_statistics_read");
    bridge.AddPublisherFromCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        ("io", "GetPeriodStatisticsWrite",
         ros_namespace + "/period_statistics_write");
}

void dvrk::connect_bridge_io(const std::string & bridge_name,
                             const std::string & io_component_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, "io",
                              io_component_name, "Configuration");
}

void dvrk::add_topics_io(mtsROSBridge & bridge,
                         const std::string & ros_namespace,
                         const std::string & arm_name,
                         const dvrk_topics_version::version CMN_UNUSED(version))
{
    bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (arm_name + "-io", "GetAnalogInputPosSI",
         ros_namespace + "/analog_input_pos_si");
    bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (arm_name + "-io", "GetPosition",
         ros_namespace + "/joint_position");
    bridge.AddPublisherFromCommandRead<prmPositionJointGet, sensor_msgs::JointState>
        (arm_name + "-io", "GetPositionActuator",
         ros_namespace + "/actuator_position");
}

void dvrk::connect_bridge_io(const std::string & bridge_name,
                             const std::string & io_component_name,
                             const std::string & arm_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, arm_name + "-io",
                              io_component_name, arm_name);
}

void dvrk::add_tf_arm(mtsROSBridge & tf_bridge,
                      const std::string & arm_name)
{
    tf_bridge.Addtf2BroadcasterFromCommandRead(arm_name, "GetPositionCartesian");
}

void dvrk::connect_tf_arm(const std::string & tf_bridge_name,
                          const std::string & arm_name,
                          const std::string & arm_component_name,
                          const std::string & arm_interface_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(tf_bridge_name, arm_name,
                              arm_component_name, arm_interface_name);
}

void dvrk::add_tf_suj(mtsROSBridge & tf_bridge,
                      const std::string & arm_name)
{
    tf_bridge.Addtf2BroadcasterFromCommandRead(arm_name + "-suj", "GetPositionCartesian");
}

void dvrk::connect_tf_suj(const std::string & tf_bridge_name,
                          const std::string & suj_component_name,
                          const std::string & arm_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(tf_bridge_name, arm_name + "-suj",
                              suj_component_name, arm_name);
}
