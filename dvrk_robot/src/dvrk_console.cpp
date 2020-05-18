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
    mConsole(mts_console)
{
    // start creating components
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // create all ROS bridges
    mBridgeName = "dvrk_ros" + node_handle->getNamespace();
    clean_namespace(mBridgeName);

    // shared publish bridge
    mtsROSBridge * pub_bridge = new mtsROSBridge(mBridgeName, publish_rate_in_seconds, node_handle);
    pub_bridge->AddIntervalStatisticsInterface();
    componentManager->AddComponent(pub_bridge);

    // get the stats bridge from base class
    stats_bridge().AddIntervalStatisticsPublisher("publishers", pub_bridge->GetName());

    if (mConsole->mHasIO) {
        dvrk::add_topics_io(*pub_bridge, "io");
    }

    const mtsIntuitiveResearchKitConsole::ArmList::iterator
        armEnd = mConsole->mArms.end();
    mtsIntuitiveResearchKitConsole::ArmList::iterator armIter;
    for (armIter = mConsole->mArms.begin();
         armIter != armEnd;
         ++armIter) {
        if (!armIter->second->mSkipROSBridge) {
            const std::string name = armIter->first;
            const std::string armNameSpace = name;
            switch (armIter->second->mType) {
            case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM:
            case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_DERIVED:
                // custom dVRK
                bridge_interface_provided_mtm(name, "Arm", armNameSpace,
                                              publish_rate_in_seconds, tf_rate_in_seconds);
                break;
            case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_GENERIC:
                // standard CRTK
                bridge_interface_provided(name, "Arm", armNameSpace,
                                          publish_rate_in_seconds, tf_rate_in_seconds);
                break;
            case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
            case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED:
                // custom dVRK
                bridge_interface_provided_ecm(name, "Arm", armNameSpace,
                                              publish_rate_in_seconds, tf_rate_in_seconds);
                if (armIter->second->mSimulation
                    == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                    dvrk::add_topics_ecm_io(*pub_bridge, armNameSpace,
                                            name);
                }
                break;
            case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
            case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED:
                // custom dVRK
                bridge_interface_provided_psm(name, "Arm", armNameSpace,
                                              publish_rate_in_seconds, tf_rate_in_seconds);
                if (armIter->second->mSimulation
                    == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                    dvrk::add_topics_psm_io(*pub_bridge, armNameSpace,
                                            name);
                }
                break;
            case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ:
                // dvrk::add_tf_suj(*tf_bridge, "PSM1");
                // dvrk::add_tf_suj(*tf_bridge, "PSM2");
                // dvrk::add_tf_suj(*tf_bridge, "PSM3");
                // dvrk::add_tf_suj(*tf_bridge, "ECM");
                // dvrk::add_topics_suj(*pub_bridge, "SUJ/PSM1", "PSM1");
                // dvrk::add_topics_suj(*pub_bridge, "SUJ/PSM2", "PSM2");
                // dvrk::add_topics_suj(*pub_bridge, "SUJ/PSM3", "PSM3");
                // dvrk::add_topics_suj(*pub_bridge, "SUJ/ECM", "ECM");
                break;
            default:
                break;
            }
        }
    }

    // PSM teleop
    const mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator
        teleopsEnd = mConsole->mTeleopsPSM.end();
    mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator teleopIter;
    for (teleopIter = mConsole->mTeleopsPSM.begin();
         teleopIter != teleopsEnd;
         ++teleopIter) {
        const std::string name = teleopIter->first;
        std::string topic_name = teleopIter->first;
        clean_namespace(topic_name);
        dvrk::add_topics_teleop_psm(*pub_bridge, topic_name, name);
    }

    // ECM teleop
    if (mConsole->mTeleopECM) {
        const std::string name = mConsole->mTeleopECM->Name();
        std::string topic_name = mConsole->mTeleopECM->Name();
        clean_namespace(topic_name);
        dvrk::add_topics_teleop_ecm(*pub_bridge, topic_name, name);
    }

    // digital inputs
    const std::string footPedalsNameSpace = "footpedals/";
    typedef mtsIntuitiveResearchKitConsole::DInputSourceType DInputSourceType;
    const DInputSourceType::const_iterator inputsEnd = mConsole->mDInputSources.end();
    DInputSourceType::const_iterator inputsIter;
    for (inputsIter = mConsole->mDInputSources.begin();
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
        m_events_bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            (requiredInterfaceName, "Button",
             footPedalsNameSpace + lowerName);
        componentManager->Connect(m_events_bridge->GetName(), requiredInterfaceName,
                                  inputsIter->second.first, inputsIter->second.second);

    }

    dvrk::add_topics_console(*pub_bridge, "console");
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
        const std::string ioComponentName = mConsole->GetArmIOComponentName(name);
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

void dvrk::console::bridge_interface_provided_arm(const std::string & _component_name,
                                                  const std::string & _interface_name,
                                                  const std::string & _ros_namespace,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all CRTK topics
    bridge_interface_provided(_component_name, _interface_name, _ros_namespace,
                              _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK arm

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _component_name + "_using_" + _interface_name;

    m_subscribers_bridge->AddSubscriberToCommandWrite<prmPositionCartesianSet, geometry_msgs::Pose>
        (_required_interface_name, "SetBaseFrame",
         _ros_namespace + "/set_base_frame");
    m_subscribers_bridge->AddSubscriberToCommandWrite<double, std_msgs::Float64>
        (_required_interface_name, "SetJointVelocityRatio",
         _ros_namespace + "/set_joint_velocity_ratio");
    m_subscribers_bridge->AddSubscriberToCommandWrite<double, std_msgs::Float64>
        (_required_interface_name, "SetJointAccelerationRatio",
         _ros_namespace + "/set_joint_acceleration_ratio");
    m_subscribers_bridge->AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_required_interface_name, "SetWrenchBodyOrientationAbsolute",
         _ros_namespace + "/set_wrench_body_orientation_absolute");
    m_subscribers_bridge->AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_required_interface_name, "SetGravityCompensation",
         _ros_namespace + "/set_gravity_compensation");
    m_subscribers_bridge->AddSubscriberToCommandWrite<prmCartesianImpedanceGains, cisst_msgs::prmCartesianImpedanceGains>
        (_required_interface_name, "SetCartesianImpedanceGains",
         _ros_namespace + "/set_cartesian_impedance_gains");

    m_events_bridge->AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_required_interface_name, "DesiredState",
         _ros_namespace + "/desired_state");
    m_events_bridge->AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_required_interface_name, "GoalReached",
         _ros_namespace + "/goal_reached");
    m_events_bridge->AddPublisherFromEventWrite<double, std_msgs::Float64>
        (_required_interface_name, "JointVelocityRatio",
         _ros_namespace + "/joint_velocity_ratio");
    m_events_bridge->AddPublisherFromEventWrite<double, std_msgs::Float64>
        (_required_interface_name, "JointAccelerationRatio",
         _ros_namespace + "/joint_acceleration_ratio");
}

void dvrk::console::bridge_interface_provided_ecm(const std::string & _component_name,
                                                  const std::string & _interface_name,
                                                  const std::string & _ros_namespace,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all dVRK topics
    bridge_interface_provided_arm(_component_name, _interface_name, _ros_namespace,
                                  _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK ECM

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _component_name + "_using_" + _interface_name;

    m_subscribers_bridge->AddSubscriberToCommandWrite<std::string, std_msgs::String>
        (_required_interface_name, "SetEndoscopeType",
         _ros_namespace + "/set_endoscope_type");

    m_events_bridge->AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_required_interface_name, "EndoscopeType",
         _ros_namespace + "/endoscope_type");
    m_events_bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
        (_required_interface_name, "ManipClutch",
         _ros_namespace + "/manip_clutch");
}

void dvrk::console::bridge_interface_provided_mtm(const std::string & _component_name,
                                                  const std::string & _interface_name,
                                                  const std::string & _ros_namespace,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all dVRK topics
    bridge_interface_provided_arm(_component_name, _interface_name, _ros_namespace,
                                  _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK MTM

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _component_name + "_using_" + _interface_name;

    m_subscribers_bridge->AddSubscriberToCommandWrite<vctMatRot3, geometry_msgs::Quaternion>
        (_required_interface_name, "LockOrientation",
         _ros_namespace + "/lock_orientation");
    m_subscribers_bridge->AddSubscriberToCommandVoid
        (_required_interface_name, "UnlockOrientation",
         _ros_namespace + "/unlock_orientation");

    m_events_bridge->AddPublisherFromEventVoid
        (_required_interface_name, "GripperPinchEvent",
         _ros_namespace + "/gripper_pinch_event");
    m_events_bridge->AddPublisherFromEventWrite<bool, std_msgs::Bool>
        (_required_interface_name, "GripperClosedEvent",
         _ros_namespace + "/gripper_closed_event");
}

void dvrk::console::bridge_interface_provided_psm(const std::string & _component_name,
                                                  const std::string & _interface_name,
                                                  const std::string & _ros_namespace,
                                                  const double _publish_period_in_seconds,
                                                  const double _tf_period_in_seconds)
{
    // first call the base class method for all dVRK topics
    bridge_interface_provided_arm(_component_name, _interface_name, _ros_namespace,
                                  _publish_period_in_seconds, _tf_period_in_seconds);

    // now the publisher for the arm should have been created, we can
    // add topics specific to dVRK PSM

    // required interface for bridges shared across components being
    // bridged (e.g. subscribers and events)
    const std::string _required_interface_name = _component_name + "_using_" + _interface_name;

    m_subscribers_bridge->AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_required_interface_name, "SetAdapterPresent",
         _ros_namespace + "/set_adapter_present");
    m_subscribers_bridge->AddSubscriberToCommandWrite<bool, std_msgs::Bool>
        (_required_interface_name, "SetToolPresent",
         _ros_namespace + "/set_tool_present");
    m_subscribers_bridge->AddSubscriberToCommandWrite<std::string, std_msgs::String>
        (_required_interface_name, "SetToolType",
         _ros_namespace + "/set_tool_type");

    m_events_bridge->AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        (_required_interface_name, "ManipClutch",
         _ros_namespace + "/manip_clutch");
    m_events_bridge->AddPublisherFromEventVoid
        (_required_interface_name, "ToolTypeRequest",
         _ros_namespace + "/tool_type_request");
    m_events_bridge->AddPublisherFromEventWrite<std::string, std_msgs::String>
        (_required_interface_name, "ToolType",
         _ros_namespace + "/tool_type");
}

void dvrk::console::Connect(void)
{
    mts_ros_crtk_bridge::Connect();

    if (mConsole->mHasIO) {
        dvrk::connect_bridge_io(mBridgeName, mConsole->mIOComponentName);
    }

    const mtsIntuitiveResearchKitConsole::ArmList::iterator
        armEnd = mConsole->mArms.end();
    mtsIntuitiveResearchKitConsole::ArmList::iterator armIter;
    for (armIter = mConsole->mArms.begin();
         armIter != armEnd;
         ++armIter) {
        if (!armIter->second->mSkipROSBridge) {
            const std::string name = armIter->first;
            switch (armIter->second->mType) {
            case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
            case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED:
                if (armIter->second->mSimulation
                    == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                    dvrk::connect_bridge_ecm_io(mBridgeName, name,
                                                armIter->second->IOComponentName());
                }
                break;
            case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
            case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED:
                if (armIter->second->mSimulation
                    == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                    dvrk::connect_bridge_psm_io(mBridgeName, name,
                                                armIter->second->IOComponentName());
                }
                break;
            case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ:
                // dvrk::connect_tf_suj(mTfBridgeName, name, "PSM1");
                // dvrk::connect_tf_suj(mTfBridgeName, name, "PSM2");
                // dvrk::connect_tf_suj(mTfBridgeName, name, "PSM3");
                // dvrk::connect_tf_suj(mTfBridgeName, name, "ECM");
                // dvrk::connect_bridge_suj(mBridgeName, name, "PSM1");
                // dvrk::connect_bridge_suj(mBridgeName, name, "PSM2");
                // dvrk::connect_bridge_suj(mBridgeName, name, "PSM3");
                // dvrk::connect_bridge_suj(mBridgeName, name, "ECM");
            default:
                break;
            }
        }
    }

    // PSM teleop
    const mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator
        teleopsEnd = mConsole->mTeleopsPSM.end();
    mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator teleopIter;
    for (teleopIter = mConsole->mTeleopsPSM.begin();
         teleopIter != teleopsEnd;
         ++teleopIter) {
        const std::string name = teleopIter->first;
        dvrk::connect_bridge_teleop_psm(mBridgeName, name);
    }

    // ECM teleop
    if (mConsole->mTeleopECM) {
        const std::string name = mConsole->mTeleopECM->Name();
        dvrk::connect_bridge_teleop_ecm(mBridgeName, name);
    }

    // connect console bridge
    dvrk::connect_bridge_console(mBridgeName, mConsole->GetName());

    // ros wrappers for IO
    const std::list<std::string>::const_iterator end = mIOInterfaces.end();
    std::list<std::string>::const_iterator iter;
    for (iter = mIOInterfaces.begin();
         iter != end;
         iter++) {
        const std::string bridgeName = bridgeNamePrefix + *iter;
        const std::string ioComponentName = mConsole->GetArmIOComponentName(*iter);
        dvrk::connect_bridge_io(bridgeName, ioComponentName, *iter);
    }
}
