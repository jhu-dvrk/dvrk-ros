/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-18

  (C) Copyright 2015-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisst_ros_crtk/mtsCISSTToROS.h>
#include <cisst_ros_crtk/mtsROSToCISST.h>

#include <dvrk_utilities/dvrk_console.h>
#include <cisst_ros_bridge/mtsROSBridge.h>

#include <cisstCommon/cmnStrings.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>

#include <json/json.h>

const std::string bridgeNamePrefix = "dVRKIOBridge";

dvrk::console::console(const std::string & name,
                       ros::NodeHandle * node_handle,
                       const double & publish_rate_in_seconds,
                       const double & tf_rate_in_seconds,
                       mtsIntuitiveResearchKitConsole * mts_console):
    mts_ros_crtk_bridge(name, node_handle),
    m_console(mts_console)
{
    // start creating components
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // create all ROS bridges
    std::string m_bridge_name = "dvrk_ros" + node_handle->getNamespace();
    clean_namespace(m_bridge_name);

    // shared publish bridge
    m_pub_bridge = new mtsROSBridge(m_bridge_name, publish_rate_in_seconds, node_handle);
    m_pub_bridge->AddIntervalStatisticsInterface();
    componentManager->AddComponent(m_pub_bridge);

    // get the stats bridge from base class
    stats_bridge().AddIntervalStatisticsPublisher("publishers", m_pub_bridge->GetName());

    if (m_console->mHasIO) {
        dvrk::add_topics_io(*m_pub_bridge, "io");
    }

    const mtsIntuitiveResearchKitConsole::ArmList::iterator
        armEnd = m_console->mArms.end();
    mtsIntuitiveResearchKitConsole::ArmList::iterator armIter;
    for (armIter = m_console->mArms.begin();
         armIter != armEnd;
         ++armIter) {
        if (!armIter->second->mSkipROSBridge) {
            const std::string name = armIter->first;
            switch (armIter->second->mType) {
            case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM:
            case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_DERIVED:
                // custom dVRK
                bridge_interface_provided_mtm(name, "Arm",
                                              publish_rate_in_seconds, tf_rate_in_seconds);
                break;
            case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_GENERIC:
                // standard CRTK
                bridge_interface_provided(name, "Arm",
                                          publish_rate_in_seconds, tf_rate_in_seconds);
                break;
            case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
            case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED:
                // custom dVRK
                bridge_interface_provided_ecm(name, "Arm",
                                              publish_rate_in_seconds, tf_rate_in_seconds);
                if (armIter->second->mSimulation
                    == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                    add_topics_ecm_io(name, armIter->second->mIOComponentName);
                }
                break;
            case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
            case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED:
                // custom dVRK
                bridge_interface_provided_psm(name, "Arm",
                                              publish_rate_in_seconds, tf_rate_in_seconds);
                if (armIter->second->mSimulation
                    == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                    add_topics_psm_io(name, armIter->second->mIOComponentName);
                }
                break;
            case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ:
                {
                    const auto _sujs = std::list<std::string>({"PSM1", "PSM2", "PSM3", "ECM"});
                    for (auto const & _suj : _sujs) {
                        bridge_interface_provided(name,
                                                  _suj,
                                                  "SUJ/" + _suj,
                                                  publish_rate_in_seconds,
                                                  tf_rate_in_seconds);
                    }
                }
                break;
            default:
                break;
            }
        }
    }


    // ECM teleop
    if (m_console->mTeleopECM) {
        add_topics_teleop_ecm(m_console->mTeleopECM->Name());
    }

    // PSM teleops
    for (auto const & teleop : m_console->mTeleopsPSM) {
        add_topics_teleop_psm(teleop.first);
    }

    // digital inputs
    const std::string footPedalsNameSpace = "footpedals/";
    typedef mtsIntuitiveResearchKitConsole::DInputSourceType DInputSourceType;
    const DInputSourceType::const_iterator inputsEnd = m_console->mDInputSources.end();
    DInputSourceType::const_iterator inputsIter;
    for (inputsIter = m_console->mDInputSources.begin();
         inputsIter != inputsEnd;
         ++inputsIter) {
        std::string upperName = inputsIter->second.second;
        std::string lowerName = inputsIter->first;
        std::string requiredInterfaceName = upperName + "_" + lowerName;
        // put everything lower case
        std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(), tolower);
        // replace +/- by strings
        cmnStringReplaceAll(lowerName, "-", "_minus");
        cmnStringReplaceAll(lowerName, "+", "_plus");
        events_bridge().AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            (requiredInterfaceName, "Button",
             footPedalsNameSpace + lowerName);
        componentManager->Connect(events_bridge().GetName(), requiredInterfaceName,
                                  inputsIter->second.first, inputsIter->second.second);

    }

    // console
    add_topics_console();
}

void dvrk::console::Configure(const std::string & jsonFile)
{
    std::ifstream jsonStream;
    jsonStream.open(jsonFile.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        std::cerr << "Configure: failed to parse configuration\n"
                  << jsonReader.getFormattedErrorMessages();
        return;
    }

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // look for io-interfaces
    const Json::Value interfaces = jsonConfig["io-interfaces"];
    for (unsigned int index = 0; index < interfaces.size(); ++index) {
        const std::string name = interfaces[index]["name"].asString();
        const double period = interfaces[index]["period"].asFloat();
        const std::string ioComponentName = m_console->GetArmIOComponentName(name);
        if (ioComponentName == "") {
            std::cerr << "Warning: the arm \"" << name << "\" doesn't seem to exist" << std::endl
                      << "or it doesn't have an IO component, no ROS bridge connected" << std::endl
                      << "for this IO." << std::endl;
        } else {
            mtsROSBridge * rosIOBridge = new mtsROSBridge(bridgeNamePrefix + name, period,
                                                          node_handle_ptr());
            dvrk::add_topics_io(*rosIOBridge,
                                name + "/io/",
                                name);
            componentManager->AddComponent(rosIOBridge);
            mIOInterfaces.push_back(name);
        }
    }
}

void dvrk::console::bridge_interface_provided_arm(const std::string & _arm_name,
                                                  const std::string & _interface_name,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all CRTK topics
    bridge_interface_provided(_arm_name, _interface_name, _arm_name,
                              _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK arm

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _arm_name + "_using_" + _interface_name;

    subscribers_bridge().AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::Pose>
        (_required_interface_name, "SetBaseFrame",
         _arm_name + "/set_base_frame");
    subscribers_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float64>
        (_required_interface_name, "SetJointVelocityRatio",
         _arm_name + "/set_joint_velocity_ratio");
    subscribers_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float64>
        (_required_interface_name, "SetJointAccelerationRatio",
         _arm_name + "/set_joint_acceleration_ratio");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_required_interface_name, "SetWrenchBodyOrientationAbsolute",
         _arm_name + "/set_wrench_body_orientation_absolute");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_required_interface_name, "SetGravityCompensation",
         _arm_name + "/set_gravity_compensation");
    subscribers_bridge().AddSubscriberToCommandWrite<prmCartesianImpedanceGains, cisst_msgs::prmCartesianImpedanceGains>
        (_required_interface_name, "SetCartesianImpedanceGains",
         _arm_name + "/set_cartesian_impedance_gains");

    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_required_interface_name, "DesiredState",
         _arm_name + "/desired_state");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_required_interface_name, "GoalReached",
         _arm_name + "/goal_reached");
    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float64>
        (_required_interface_name, "JointVelocityRatio",
         _arm_name + "/joint_velocity_ratio");
    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float64>
        (_required_interface_name, "JointAccelerationRatio",
         _arm_name + "/joint_acceleration_ratio");
}

void dvrk::console::bridge_interface_provided_ecm(const std::string & _arm_name,
                                                  const std::string & _interface_name,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all dVRK topics
    bridge_interface_provided_arm(_arm_name, _interface_name,
                                  _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK ECM

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _arm_name + "_using_" + _interface_name;

    subscribers_bridge().AddSubscriberToCommandWrite<std::string, std_msgs::String>
        (_required_interface_name, "SetEndoscopeType",
         _arm_name + "/set_endoscope_type");

    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_required_interface_name, "EndoscopeType",
         _arm_name + "/endoscope_type");
    events_bridge().AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        (_required_interface_name, "ManipClutch",
         _arm_name + "/manip_clutch");
}

void dvrk::console::bridge_interface_provided_mtm(const std::string & _arm_name,
                                                  const std::string & _interface_name,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all dVRK topics
    bridge_interface_provided_arm(_arm_name, _interface_name,
                                  _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK MTM

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _arm_name + "_using_" + _interface_name;

    subscribers_bridge().AddSubscriberToCommandWrite<vctMatRot3, geometry_msgs::Quaternion>
        (_required_interface_name, "LockOrientation",
         _arm_name + "/lock_orientation");
    subscribers_bridge().AddSubscriberToCommandVoid
        (_required_interface_name, "UnlockOrientation",
         _arm_name + "/unlock_orientation");

    events_bridge().AddPublisherFromEventVoid
        (_required_interface_name, "GripperPinchEvent",
         _arm_name + "/gripper_pinch_event");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_required_interface_name, "GripperClosedEvent",
         _arm_name + "/gripper_closed_event");
}

void dvrk::console::bridge_interface_provided_psm(const std::string & _arm_name,
                                                  const std::string & _interface_name,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all dVRK topics
    bridge_interface_provided_arm(_arm_name, _interface_name,
                                  _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK PSM

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _arm_name + "_using_" + _interface_name;

    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_required_interface_name, "SetAdapterPresent",
         _arm_name + "/set_adapter_present");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_required_interface_name, "SetToolPresent",
         _arm_name + "/set_tool_present");
    subscribers_bridge().AddSubscriberToCommandWrite<std::string, std_msgs::String>
        (_required_interface_name, "SetToolType",
         _arm_name + "/set_tool_type");

    events_bridge().AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        (_required_interface_name, "ManipClutch",
         _arm_name + "/manip_clutch");
    events_bridge().AddPublisherFromEventVoid
        (_required_interface_name, "ToolTypeRequest",
         _arm_name + "/tool_type_request");
    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_required_interface_name, "ToolType",
         _arm_name + "/tool_type");
}

void dvrk::console::add_topics_console(void)
{
    const std::string _ros_namespace = "console";

    subscribers_bridge().AddSubscriberToCommandVoid
        ("Console", "PowerOff",
         _ros_namespace + "/power_off");
    subscribers_bridge().AddSubscriberToCommandVoid
        ("Console", "PowerOn",
         _ros_namespace + "/power_on");
    subscribers_bridge().AddSubscriberToCommandVoid
        ("Console", "Home",
         _ros_namespace + "/home");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        ("Console", "TeleopEnable",
         _ros_namespace + "/teleop/enable");
    subscribers_bridge().AddSubscriberToCommandWrite<std::string, std_msgs::String>
        ("Console", "CycleTeleopPSMByMTM",
         _ros_namespace + "/teleop/cycle_teleop_psm_by_mtm");
    subscribers_bridge().AddSubscriberToCommandWrite<prmKeyValue, diagnostic_msgs::KeyValue>
        ("Console", "SelectTeleopPSM",
         _ros_namespace + "/teleop/select_teleop_psm");
    subscribers_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float32>
        ("Console", "SetScale",
         _ros_namespace + "/teleop/set_scale");

    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float32>
        ("Console", "Scale",
         _ros_namespace + "/teleop/scale");
    events_bridge().AddPublisherFromEventWrite<prmKeyValue, diagnostic_msgs::KeyValue>
        ("Console", "TeleopPSMSelected",
         _ros_namespace + "/teleop/teleop_psm_selected");
    events_bridge().AddPublisherFromEventWrite<prmKeyValue, diagnostic_msgs::KeyValue>
        ("Console", "TeleopPSMUnselected",
         _ros_namespace + "/teleop/teleop_psm_unselected");
    events_bridge().AddSubscriberToCommandWrite<std::string, std_msgs::String>
        ("Console", "StringToSpeech",
         _ros_namespace + "/string_to_speech");

    m_connections.Add(subscribers_bridge().GetName(), "Console",
                      m_console->GetName(), "Main");
    m_connections.Add(events_bridge().GetName(), "Console",
                      m_console->GetName(), "Main");
}

void dvrk::console::add_topics_ecm_io(const std::string & _arm_name,
                                      const std::string & _io_component_name)
{
    // known events and corresponding ros topic
    const auto events = std::list<std::pair<std::string, std::string> >({
            {"ManipClutch", "manip_clutch"},
                {"SUJClutch", "suj_clutch"}});
    for (auto event = events.begin();
         event != events.end();
         ++event) {
        std::string _interface_name = _arm_name + "-" + event->first;
        events_bridge().AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            (_interface_name, "Button", _arm_name + "/io/" + event->second);
        m_connections.Add(events_bridge().GetName(), _interface_name,
                          _io_component_name, _interface_name);
    }
}

void dvrk::console::add_topics_psm_io(const std::string & _arm_name,
                                      const std::string & _io_component_name)
{
    // known events and corresponding ros topic
    const auto events = std::list<std::pair<std::string, std::string> >({
            {"ManipClutch", "manip_clutch"},
                {"SUJClutch", "suj_clutch"},
                    {"Adapter", "adapter"},
                        {"Tool", "tool"}});
    for (auto event = events.begin();
         event != events.end();
         ++event) {
        std::string _interface_name = _arm_name + "-" + event->first;
        events_bridge().AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            (_interface_name, "Button", _arm_name + "/io/" + event->second);
        m_connections.Add(events_bridge().GetName(), _interface_name,
                          _io_component_name, _interface_name);
    }
}

void dvrk::console::add_topics_teleop_ecm(const std::string & _name)
{
    std::string _ros_namespace = _name;
    clean_namespace(_ros_namespace);

    // messages
    events_bridge().AddLogFromEventWrite(_name + "-log", "error",
                                         mtsROSEventWriteLog::ROS_LOG_ERROR);
    events_bridge().AddLogFromEventWrite(_name + "-log", "warning",
                                         mtsROSEventWriteLog::ROS_LOG_WARN);
    events_bridge().AddLogFromEventWrite(_name + "-log", "status",
                                         mtsROSEventWriteLog::ROS_LOG_INFO);
    // connect
    m_connections.Add(events_bridge().GetName(), _name + "-log",
                      _name, "Setting");

    // events
    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_name, "DesiredState", _ros_namespace + "/desired_state");
    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_name, "CurrentState", _ros_namespace + "/current_state");
    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float32>
        (_name, "Scale", _ros_namespace + "/scale");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_name, "Following", _ros_namespace + "/following");
    // connect
    m_connections.Add(events_bridge().GetName(), _name,
                      _name, "Setting");

    // commands
    subscribers_bridge().AddSubscriberToCommandWrite<std::string, crtk_msgs::StringStamped>
        (_name, "state_command",
         _ros_namespace + "/state_command");
    subscribers_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float32>
        (_name, "SetScale",
         _ros_namespace + "/set_scale");
    // connect
    m_connections.Add(subscribers_bridge().GetName(), _name,
                      _name, "Setting");
}

void dvrk::console::add_topics_teleop_psm(const std::string & _name)
{
    std::string _ros_namespace = _name;
    clean_namespace(_ros_namespace);

    // messages
    events_bridge().AddLogFromEventWrite(_name + "-log", "error",
                                         mtsROSEventWriteLog::ROS_LOG_ERROR);
    events_bridge().AddLogFromEventWrite(_name + "-log", "warning",
                                         mtsROSEventWriteLog::ROS_LOG_WARN);
    events_bridge().AddLogFromEventWrite(_name + "-log", "status",
                                         mtsROSEventWriteLog::ROS_LOG_INFO);
    // connect
    m_connections.Add(events_bridge().GetName(), _name + "-log",
                      _name, "Setting");

    // publisher
    m_pub_bridge->AddPublisherFromCommandRead<vctMatRot3, geometry_msgs::QuaternionStamped>
        (_name, "GetAlignOffset",
         _ros_namespace + "/align_offset");
    // connect
    m_connections.Add(m_pub_bridge->GetName(), _name,
                      _name, "Setting");

    // events
    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_name, "DesiredState", _ros_namespace + "/desired_state");
    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_name, "CurrentState", _ros_namespace + "/current_state");
    events_bridge().AddPublisherFromEventWrite<bool, sensor_msgs::Joy>
        (_name, "RotationLocked",
         _ros_namespace + "/rotation_locked");
    events_bridge().AddPublisherFromEventWrite<bool, sensor_msgs::Joy>
        (_name, "TranslationLocked",
         _ros_namespace + "/translation_locked");
    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float32>
        (_name, "Scale", _ros_namespace + "/scale");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_name, "Following", _ros_namespace + "/following");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_name, "AlignMTM", _ros_namespace + "/align_mtm");
    // connect
    m_connections.Add(events_bridge().GetName(), _name,
                      _name, "Setting");

    // commands
    subscribers_bridge().AddSubscriberToCommandWrite<std::string, crtk_msgs::StringStamped>
        (_name, "state_command",
         _ros_namespace + "/state_command");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_name, "LockTranslation",
         _ros_namespace + "/lock_translation");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_name, "LockRotation",
         _ros_namespace + "/lock_rotation");
    subscribers_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float32>
        (_name, "SetScale",
         _ros_namespace + "/set_scale");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_name, "SetAlignMTM",
         _ros_namespace + "/set_align_mtm");
    subscribers_bridge().AddSubscriberToCommandWrite<vctMatRot3, geometry_msgs::Quaternion>
        (_name, "SetRegistrationRotation",
         _ros_namespace + "/set_registration_rotation");
    // connect
    m_connections.Add(subscribers_bridge().GetName(), _name,
                      _name, "Setting");
}

void dvrk::console::Connect(void)
{
    mts_ros_crtk_bridge::Connect();

    if (m_console->mHasIO) {
        dvrk::connect_bridge_io(m_pub_bridge->GetName(), m_console->mIOComponentName);
    }

    // ros wrappers for IO
    const std::list<std::string>::const_iterator end = mIOInterfaces.end();
    std::list<std::string>::const_iterator iter;
    for (iter = mIOInterfaces.begin();
         iter != end;
         iter++) {
        const std::string bridgeName = bridgeNamePrefix + *iter;
        const std::string ioComponentName = m_console->GetArmIOComponentName(*iter);
        dvrk::connect_bridge_io(bridgeName, ioComponentName, *iter);
    }
}
