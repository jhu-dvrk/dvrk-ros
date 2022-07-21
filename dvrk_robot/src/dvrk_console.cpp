/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-18

  (C) Copyright 2015-2022 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisst_ros_crtk/cisst_ros_crtk.h>

#include <cisstCommon/cmnStrings.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>
#include <sawIntuitiveResearchKit/mtsDaVinciEndoscopeFocus.h>

#include <json/json.h>

const std::string bridgeNamePrefix = "dVRKIOBridge_";

CMN_IMPLEMENT_SERVICES(dvrk_console);

dvrk::console::console(const std::string & name,
                       ros::NodeHandle * node_handle,
                       const double & publish_rate_in_seconds,
                       const double & tf_rate_in_seconds,
                       mtsIntuitiveResearchKitConsole * mts_console):
    mts_ros_crtk_bridge_provided(name, node_handle),
    m_console(mts_console)
{
    // start creating components
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // create all ROS bridges
    std::string m_bridge_name = "dvrk_ros" + node_handle->getNamespace();
    cisst_ros_crtk::clean_namespace(m_bridge_name);

    // shared publish bridge
    m_pub_bridge = new mtsROSBridge(m_bridge_name, publish_rate_in_seconds, node_handle);
    m_pub_bridge->AddIntervalStatisticsInterface();
    componentManager->AddComponent(m_pub_bridge);

    // get the stats bridge from base class
    stats_bridge().AddIntervalStatisticsPublisher("publishers", m_pub_bridge->GetName());

    // IO topics
    if (m_console->mHasIO) {
        add_topics_io();
    }

    // arm topics
    for (auto armPair : m_console->mArms) {
        auto arm = *(armPair.second);
        if (!arm.m_skip_ROS_bridge) {
            const std::string name = armPair.first;
            if (arm.native_or_derived_mtm()) {
                bridge_interface_provided_mtm(name, "Arm",
                                              publish_rate_in_seconds, tf_rate_in_seconds);
            } else if (arm.native_or_derived_ecm()) {
                bridge_interface_provided_ecm(name, "Arm",
                                              publish_rate_in_seconds, tf_rate_in_seconds);
                if (arm.m_simulation
                    == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                    add_topics_ecm_io(name, arm.m_IO_component_name);
                }
            } else if (arm.native_or_derived_psm()) {
                bridge_interface_provided_psm(name, "Arm",
                                              publish_rate_in_seconds, tf_rate_in_seconds);
                if (arm.m_simulation
                    == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                    add_topics_psm_io(name, arm.m_IO_component_name);
                }
            } else if (arm.generic()) {
                bridge_interface_provided(arm.ComponentName(),
                                          arm.InterfaceName(),
                                          publish_rate_in_seconds, tf_rate_in_seconds);
            } else if (arm.m_type == mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ) {
                const auto _sujs = std::list<std::string>({"PSM1", "PSM2", "PSM3", "ECM"});
                for (auto const & _suj : _sujs) {
                    bridge_interface_provided(name,
                                              _suj,
                                              "SUJ/" + _suj,
                                              publish_rate_in_seconds,
                                              tf_rate_in_seconds);
                }
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

    // Endoscope focus
    if (m_console->mDaVinciEndoscopeFocus) {
        add_topics_endoscope_focus();
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

    mtsManagerLocal * _component_manager = mtsManagerLocal::GetInstance();

    // look for io-interfaces
    const Json::Value interfaces = jsonConfig["io-interfaces"];
    for (unsigned int index = 0; index < interfaces.size(); ++index) {
        const std::string _name = interfaces[index]["name"].asString();
        const double _period = interfaces[index]["period"].asFloat();
        const std::string _io_component_name = m_console->GetArmIOComponentName(_name);
        if (_io_component_name == "") {
            std::cerr << "Warning: the arm \"" << _name << "\" doesn't seem to exist" << std::endl
                      << "or it doesn't have an IO component, no ROS bridge connected" << std::endl
                      << "for this IO." << std::endl;
        } else {
            const std::string _bridgeName = bridgeNamePrefix + _name;
            mtsROSBridge * _rosIOBridge = new mtsROSBridge(bridgeNamePrefix + _name, _period,
                                                           node_handle_ptr());
            _component_manager->AddComponent(_rosIOBridge);
            add_topics_arm_io(_rosIOBridge, _name, _io_component_name);
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
        (_required_interface_name, "set_base_frame",
         _arm_name + "/set_base_frame");
    subscribers_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float64>
        (_required_interface_name, "trajectory_j/set_ratio",
         _arm_name + "/trajectory_j/set_ratio");
    subscribers_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float64>
        (_required_interface_name, "trajectory_j/set_ratio_v",
         _arm_name + "/trajectory_j/set_ratio_v");
    subscribers_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float64>
        (_required_interface_name, "trajectory_j/set_ratio_a",
         _arm_name + "/trajectory_j/set_ratio_a");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_required_interface_name, "body/set_cf_orientation_absolute",
         _arm_name + "/body/set_cf_orientation_absolute");
    subscribers_bridge().AddSubscriberToCommandWrite<prmCartesianImpedanceGains, cisst_msgs::prmCartesianImpedanceGains>
        (_required_interface_name, "set_cartesian_impedance_gains",
         _arm_name + "/set_cartesian_impedance_gains");

    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_required_interface_name, "desired_state",
         _arm_name + "/desired_state");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_required_interface_name, "goal_reached",
         _arm_name + "/goal_reached");
    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float64>
        (_required_interface_name, "trajectory_j/ratio",
         _arm_name + "/trajectory_j/ratio");
    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float64>
        (_required_interface_name, "trajectory_j/ratio_v",
         _arm_name + "/trajectory_j/ratio_v");
    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float64>
        (_required_interface_name, "trajectory_j/ratio_a",
         _arm_name + "/trajectory_j/ratio_a");
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
        (_required_interface_name, "set_endoscope_type",
         _arm_name + "/set_endoscope_type");

    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_required_interface_name, "endoscope_type",
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
        (_required_interface_name, "lock_orientation",
         _arm_name + "/lock_orientation");
    subscribers_bridge().AddSubscriberToCommandVoid
        (_required_interface_name, "unlock_orientation",
         _arm_name + "/unlock_orientation");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_required_interface_name, "orientation_locked",
         _arm_name + "/orientation_locked");

    events_bridge().AddPublisherFromEventVoid
        (_required_interface_name, "gripper/pinch",
         _arm_name + "/gripper/pinch");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_required_interface_name, "gripper/closed",
         _arm_name + "/gripper/closed");
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
        (_required_interface_name, "emulate_adapter_present",
         _arm_name + "/emulate_adapter_present");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_required_interface_name, "emulate_tool_present",
         _arm_name + "/emulate_tool_present");
    subscribers_bridge().AddSubscriberToCommandWrite<std::string, std_msgs::String>
        (_required_interface_name, "set_tool_type",
         _arm_name + "/set_tool_type");

    events_bridge().AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        (_required_interface_name, "ManipClutch",
         _arm_name + "/manip_clutch");
    events_bridge().AddPublisherFromEventVoid
        (_required_interface_name, "tool_type_request",
         _arm_name + "/tool_type_request");
    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_required_interface_name, "tool_type",
         _arm_name + "/tool_type");
}

void dvrk::console::add_topics_console(void)
{
    const std::string _ros_namespace = "console/";

    subscribers_bridge().AddSubscriberToCommandVoid
        ("Console", "power_off",
         _ros_namespace + "power_off");
    subscribers_bridge().AddSubscriberToCommandVoid
        ("Console", "power_on",
         _ros_namespace + "power_on");
    subscribers_bridge().AddSubscriberToCommandVoid
        ("Console", "home",
         _ros_namespace + "home");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        ("Console", "teleop_enable",
         _ros_namespace + "teleop/enable");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        ("Console", "teleop_enabled",
         _ros_namespace + "teleop/enabled");

    subscribers_bridge().AddSubscriberToCommandWrite<std::string, std_msgs::String>
        ("Console", "cycle_teleop_psm_by_mtm",
         _ros_namespace + "teleop/cycle_teleop_psm_by_mtm");
    subscribers_bridge().AddSubscriberToCommandWrite<prmKeyValue, diagnostic_msgs::KeyValue>
        ("Console", "select_teleop_psm",
         _ros_namespace + "teleop/select_teleop_psm");
    subscribers_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float64>
        ("Console", "set_scale",
         _ros_namespace + "teleop/set_scale");

    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float64>
        ("Console", "scale",
         _ros_namespace + "/teleop/scale");
    events_bridge().AddPublisherFromEventWrite<prmKeyValue, diagnostic_msgs::KeyValue>
        ("Console", "teleop_psm_selected",
         _ros_namespace + "/teleop/teleop_psm_selected");
    events_bridge().AddPublisherFromEventWrite<prmKeyValue, diagnostic_msgs::KeyValue>
        ("Console", "teleop_psm_unselected",
         _ros_namespace + "/teleop/teleop_psm_unselected");

    events_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float64>
        ("Console", "set_volume",
         _ros_namespace + "/set_volume");
    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float64>
        ("Console", "volume",
         _ros_namespace + "/volume");
    events_bridge().AddSubscriberToCommandWrite<vctDoubleVec, std_msgs::Float64MultiArray>
        ("Console", "beep",
         _ros_namespace + "/beep");
    events_bridge().AddSubscriberToCommandWrite<std::string, std_msgs::String>
        ("Console", "string_to_speech",
         _ros_namespace + "/string_to_speech");

    events_bridge().AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        ("ConsoleOperatorPresent", "Button",
         _ros_namespace + "/operator_present");
    m_connections.Add(events_bridge().GetName(), "ConsoleOperatorPresent",
                      m_console->GetName(), "OperatorPresent");
    events_bridge().AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        ("ConsoleClutch", "Button",
         _ros_namespace + "/clutch");
    m_connections.Add(events_bridge().GetName(), "ConsoleClutch",
                      m_console->GetName(), "Clutch");
    events_bridge().AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        ("ConsoleCamera", "Button",
         _ros_namespace + "/camera");
    m_connections.Add(events_bridge().GetName(), "ConsoleCamera",
                      m_console->GetName(), "Camera");

    subscribers_bridge().AddSubscriberToCommandWrite<prmEventButton, sensor_msgs::Joy>
        ("Console", "emulate_operator_present",
         _ros_namespace + "/emulate_operator_present");
    subscribers_bridge().AddSubscriberToCommandWrite<prmEventButton, sensor_msgs::Joy>
        ("Console", "emulate_clutch",
         _ros_namespace + "/emulate_clutch");
    subscribers_bridge().AddSubscriberToCommandWrite<prmEventButton, sensor_msgs::Joy>
        ("Console", "emulate_camera",
         _ros_namespace + "/emulate_camera");

    m_connections.Add(subscribers_bridge().GetName(), "Console",
                      m_console->GetName(), "Main");
    m_connections.Add(events_bridge().GetName(), "Console",
                      m_console->GetName(), "Main");
}

void dvrk::console::add_topics_endoscope_focus(void)
{
    const std::string _ros_namespace = "endoscope_focus/";
    const std::string _focus_component_name = m_console->mDaVinciEndoscopeFocus->GetName();

    // events
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_focus_component_name, "locked",
         _ros_namespace + "/locked");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_focus_component_name, "focusing_in",
         _ros_namespace + "/focusing_in");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_focus_component_name, "focusing_out",
         _ros_namespace + "/focusing_out");

    // commands
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_focus_component_name, "lock",
         _ros_namespace + "/lock");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_focus_component_name, "focus_in",
         _ros_namespace + "/focus_in");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_focus_component_name, "focus_out",
         _ros_namespace + "/focus_out");

    m_connections.Add(subscribers_bridge().GetName(), _focus_component_name,
                      _focus_component_name, "Control");
    m_connections.Add(events_bridge().GetName(), _focus_component_name,
                      _focus_component_name, "Control");
}

void dvrk::console::add_topics_io(void)
{
    const std::string _ros_namespace = "stats/io/";
    m_pub_bridge->AddPublisherFromCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        ("io", "period_statistics",
         _ros_namespace + "period_statistics");
    m_pub_bridge->AddPublisherFromCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        ("io", "period_statistics_read",
         _ros_namespace + "period_statistics_read");
    m_pub_bridge->AddPublisherFromCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        ("io", "period_statistics_write",
         _ros_namespace + "period_statistics_write");

    m_connections.Add(m_pub_bridge->GetName(), "io",
                      m_console->m_IO_component_name, "Configuration");
}

void dvrk::console::add_topics_arm_io(mtsROSBridge * _pub_bridge,
                                      const std::string & _arm_name,
                                      const std::string & _io_component_name)
{
    const std::string _ros_namespace = _arm_name + "/io/";
    const std::string _interface_name = _arm_name + "-io";
    _pub_bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        (_interface_name, "GetAnalogInputPosSI",
         _ros_namespace + "pot/measured_js");
    _pub_bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        (_interface_name, "measured_js",
         _ros_namespace + "joint/measured_js");
    _pub_bridge->AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        (_interface_name, "actuator_measured_js",
         _ros_namespace + "actuator/measured_js");
    _pub_bridge->AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (_interface_name, "GetActuatorFeedbackCurrent",
         _ros_namespace + "actuator/measured_current");
    _pub_bridge->AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (_interface_name, "GetActuatorRequestedCurrent",
         _ros_namespace + "actuator/servo_current");

    m_connections.Add(_pub_bridge->GetName(), _interface_name,
                      _io_component_name, _arm_name);
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
    cisst_ros_crtk::clean_namespace(_ros_namespace);

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
        (_name, "desired_state", _ros_namespace + "/desired_state");
    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_name, "current_state", _ros_namespace + "/current_state");
    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float64>
        (_name, "scale", _ros_namespace + "/scale");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_name, "following", _ros_namespace + "/following");
    // connect
    m_connections.Add(events_bridge().GetName(), _name,
                      _name, "Setting");

    // commands
    subscribers_bridge().AddSubscriberToCommandWrite<std::string, crtk_msgs::StringStamped>
        (_name, "state_command",
         _ros_namespace + "/state_command");
    subscribers_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float64>
        (_name, "set_scale",
         _ros_namespace + "/set_scale");
    // connect
    m_connections.Add(subscribers_bridge().GetName(), _name,
                      _name, "Setting");
}

void dvrk::console::add_topics_teleop_psm(const std::string & _name)
{
    std::string _ros_namespace = _name;
    cisst_ros_crtk::clean_namespace(_ros_namespace);

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
        (_name, "alignment_offset",
         _ros_namespace + "/alignment_offset");
    // connect
    m_connections.Add(m_pub_bridge->GetName(), _name,
                      _name, "Setting");

    // events
    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_name, "desired_state", _ros_namespace + "/desired_state");
    events_bridge().AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_name, "current_state", _ros_namespace + "/current_state");
    events_bridge().AddPublisherFromEventWrite<bool, sensor_msgs::Joy>
        (_name, "rotation_locked",
         _ros_namespace + "/rotation_locked");
    events_bridge().AddPublisherFromEventWrite<bool, sensor_msgs::Joy>
        (_name, "translation_locked",
         _ros_namespace + "/translation_locked");
    events_bridge().AddPublisherFromEventWrite<double, std_msgs::Float64>
        (_name, "scale", _ros_namespace + "/scale");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_name, "following", _ros_namespace + "/following");
    events_bridge().AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_name, "align_mtm", _ros_namespace + "/align_mtm");
    // connect
    m_connections.Add(events_bridge().GetName(), _name,
                      _name, "Setting");

    // commands
    subscribers_bridge().AddSubscriberToCommandWrite<std::string, crtk_msgs::StringStamped>
        (_name, "state_command",
         _ros_namespace + "/state_command");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_name, "lock_translation",
         _ros_namespace + "/lock_translation");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_name, "lock_rotation",
         _ros_namespace + "/lock_rotation");
    subscribers_bridge().AddSubscriberToCommandWrite<double, std_msgs::Float64>
        (_name, "set_scale",
         _ros_namespace + "/set_scale");
    subscribers_bridge().AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_name, "set_align_mtm",
         _ros_namespace + "/set_align_mtm");
    subscribers_bridge().AddSubscriberToCommandWrite<vctMatRot3, geometry_msgs::Quaternion>
        (_name, "set_registration_rotation",
         _ros_namespace + "/set_registration_rotation");
    // connect
    m_connections.Add(subscribers_bridge().GetName(), _name,
                      _name, "Setting");
}
