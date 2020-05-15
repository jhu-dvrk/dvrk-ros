/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2015-04-33

  (C) Copyright 2015-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisst_ros_crtk/mtsCISSTToROS.h>
#include <cisst_ros_crtk/mtsROSToCISST.h>

#include <dvrk_utilities/dvrk_add_topics_functions.h>

void dvrk::add_topics_console(mtsROSBridge & bridge,
                              const std::string & ros_namespace)
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
    bridge.AddSubscriberToCommandWrite<std::string, std_msgs::String>
        ("Console", "StringToSpeech",
         ros_namespace + "/string_to_speech");
}

void dvrk::connect_bridge_console(const std::string & bridge_name,
                                  const std::string & console_component_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, "Console",
                              console_component_name, "Main");
}

void dvrk::add_topics_footpedals(mtsROSBridge & bridge,
                                 const std::string & ros_namespace)
{
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

void dvrk::add_topics_psm_io(mtsROSBridge & bridge,
                             const std::string & ros_namespace,
                             const std::string & arm_name)
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

void dvrk::add_topics_ecm_io(mtsROSBridge & bridge,

                             const std::string & ros_namespace,
                             const std::string & arm_name)
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

void dvrk::add_topics_teleop_psm(mtsROSBridge & bridge,
                                 const std::string & ros_namespace,
                                 const std::string & teleop_component_name)
{
    // messages
    bridge.AddLogFromEventWrite(teleop_component_name + "-log", "error",
                                mtsROSEventWriteLog::ROS_LOG_ERROR);
    bridge.AddLogFromEventWrite(teleop_component_name + "-log", "warning",
                                mtsROSEventWriteLog::ROS_LOG_WARN);
    bridge.AddLogFromEventWrite(teleop_component_name + "-log", "status",
                                mtsROSEventWriteLog::ROS_LOG_INFO);

    // publisher
    bridge.AddPublisherFromCommandRead<vctMatRot3, geometry_msgs::QuaternionStamped>
        (teleop_component_name, "GetAlignOffset",
         ros_namespace + "/align_offset");

    // events
    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (teleop_component_name, "DesiredState", ros_namespace + "/desired_state");
    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (teleop_component_name, "CurrentState", ros_namespace + "/current_state");
    bridge.AddPublisherFromEventWrite<bool, sensor_msgs::Joy>
        (teleop_component_name, "RotationLocked",
         ros_namespace + "/rotation_locked");
    bridge.AddPublisherFromEventWrite<bool, sensor_msgs::Joy>
        (teleop_component_name, "TranslationLocked",
         ros_namespace + "/translation_locked");
    bridge.AddPublisherFromEventWrite<double, std_msgs::Float32>
        (teleop_component_name, "Scale", ros_namespace + "/scale");
    bridge.AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (teleop_component_name, "Following", ros_namespace + "/following");
    bridge.AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (teleop_component_name, "AlignMTM", ros_namespace + "/align_mtm");

    // commands
    bridge.AddSubscriberToCommandWrite<std::string, crtk_msgs::StringStamped>
        (teleop_component_name, "state_command",
         ros_namespace + "/state_command");
    bridge.AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (teleop_component_name, "LockTranslation",
         ros_namespace + "/lock_translation");
    bridge.AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (teleop_component_name, "LockRotation",
         ros_namespace + "/lock_rotation");
    bridge.AddSubscriberToCommandWrite<double, std_msgs::Float32>
        (teleop_component_name, "SetScale",
         ros_namespace + "/set_scale");
    bridge.AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (teleop_component_name, "SetAlignMTM",
         ros_namespace + "/set_align_mtm");
    bridge.AddSubscriberToCommandWrite<vctMatRot3, geometry_msgs::Quaternion>
        (teleop_component_name, "SetRegistrationRotation",
         ros_namespace + "/set_registration_rotation");
}

void dvrk::connect_bridge_teleop_psm(const std::string & bridge_name,
                                     const std::string & teleop_component_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, teleop_component_name,
                              teleop_component_name, "Setting");
    componentManager->Connect(bridge_name, teleop_component_name + "-log",
                              teleop_component_name, "Setting");
}

void dvrk::add_topics_teleop_ecm(mtsROSBridge & bridge,
                                 const std::string & ros_namespace,
                                 const std::string & teleop_component_name)
{
    // messages
    bridge.AddLogFromEventWrite(teleop_component_name + "-log", "error",
                                mtsROSEventWriteLog::ROS_LOG_ERROR);
    bridge.AddLogFromEventWrite(teleop_component_name + "-log", "warning",
                                mtsROSEventWriteLog::ROS_LOG_WARN);
    bridge.AddLogFromEventWrite(teleop_component_name + "-log", "status",
                                mtsROSEventWriteLog::ROS_LOG_INFO);

    // events
    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (teleop_component_name, "DesiredState", ros_namespace + "/desired_state");
    bridge.AddPublisherFromEventWrite<std::string, std_msgs::String>
        (teleop_component_name, "CurrentState", ros_namespace + "/current_state");
    bridge.AddPublisherFromEventWrite<double, std_msgs::Float32>
        (teleop_component_name, "Scale", ros_namespace + "/scale");
    bridge.AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (teleop_component_name, "Following", ros_namespace + "/following");

    // commands
    bridge.AddSubscriberToCommandWrite<std::string, crtk_msgs::StringStamped>
        (teleop_component_name, "state_command",
         ros_namespace + "/state_command");
    bridge.AddSubscriberToCommandWrite<double, std_msgs::Float32>
        (teleop_component_name, "SetScale",
         ros_namespace + "/set_scale");
}

void dvrk::connect_bridge_teleop_ecm(const std::string & bridge_name,
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
                          const std::string & arm_name)
{
    // events
    bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        (arm_name + "-suj", "measured_js",
         ros_namespace + "/measured_js");
    bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
        (arm_name + "-suj", "servo_jp",
         ros_namespace + "/servo_jp");
    bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
        (arm_name + "-suj", "measured_cp_local",
         ros_namespace + "/local/measured_cp");
    bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
        (arm_name + "-suj", "measured_cp",
         ros_namespace + "/measured_cp");

    // messages
    bridge.AddLogFromEventWrite(arm_name + "-suj-log", "error",
                                mtsROSEventWriteLog::ROS_LOG_ERROR);
    bridge.AddLogFromEventWrite(arm_name + "-suj-log", "warning",
                                mtsROSEventWriteLog::ROS_LOG_WARN);
    bridge.AddLogFromEventWrite(arm_name + "-suj-log", "status",
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
                         const std::string & ros_namespace)
{
    bridge.AddPublisherFromCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        ("io", "period_statistics",
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
                         const std::string & arm_name)
{
    bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (arm_name + "-io", "GetAnalogInputPosSI",
         ros_namespace + "/analog_input_pos_si");
    bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        (arm_name + "-io", "measured_js",
         ros_namespace + "/joint_measured_js");
    bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        (arm_name + "-io", "actuator_measured_js",
         ros_namespace + "/actuator_measured_js");
    bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (arm_name + "-io", "GetActuatorFeedbackCurrent",
         ros_namespace + "/actuator_current_measured");
    bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (arm_name + "-io", "GetActuatorRequestedCurrent",
         ros_namespace + "/actuator_current_requested");
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
    tf_bridge.Addtf2BroadcasterFromCommandRead(arm_name, "measured_cp");
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
    tf_bridge.Addtf2BroadcasterFromCommandRead(arm_name + "-suj", "measured_cp");
}

void dvrk::connect_tf_suj(const std::string & tf_bridge_name,
                          const std::string & suj_component_name,
                          const std::string & arm_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(tf_bridge_name, arm_name + "-suj",
                              suj_component_name, arm_name);
}
