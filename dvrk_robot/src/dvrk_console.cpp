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

dvrk::console::console(const double & publish_rate_in_seconds,
                       const double & tf_rate_in_seconds,
                       const std::string & ros_namespace,
                       mtsIntuitiveResearchKitConsole * mts_console,
                       const dvrk_topics_version::version version):
    mNameSpace(ros_namespace),
    mConsole(mts_console),
    mVersion(version)
{
    // start creating components
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // create all ROS bridges
    std::string bridgeName = "sawIntuitiveResearchKit" + ros_namespace;
    bridgeName = ros::names::clean(bridgeName);
    std::replace(bridgeName.begin(), bridgeName.end(), '/', '_');
    std::replace(bridgeName.begin(), bridgeName.end(), '-', '_');
    std::replace(bridgeName.begin(), bridgeName.end(), '.', '_');

    // publish bridge
    mtsROSBridge * pub_bridge = new mtsROSBridge(bridgeName, publish_rate_in_seconds, false, false); // don't spin, don't catch sigint
    pub_bridge->AddIntervalStatisticsInterface();
    // bridge for tf
    mtsROSBridge * tf_bridge = new mtsROSBridge(bridgeName + "_tf2", tf_rate_in_seconds, false, false);
    tf_bridge->AddIntervalStatisticsInterface();
    // separate thread to spin, i.e. subscribe
    mtsROSBridge * spin_bridge = new mtsROSBridge(bridgeName + "_spin", 0.1 * cmn_ms, true, false);
    spin_bridge->AddIntervalStatisticsInterface();
    // bridge to publish stats
    mtsROSBridge * stats_bridge = new mtsROSBridge(bridgeName + "_stats", 200.0 * cmn_ms, false, false);

    componentManager->AddComponent(pub_bridge);
    componentManager->AddComponent(tf_bridge);
    componentManager->AddComponent(spin_bridge);
    componentManager->AddComponent(stats_bridge);

    stats_bridge->AddIntervalStatisticsPublisher(ros_namespace + "publishers", pub_bridge->GetName());
    stats_bridge->AddIntervalStatisticsPublisher(ros_namespace + "tf_broadcast", tf_bridge->GetName());
    stats_bridge->AddIntervalStatisticsPublisher(ros_namespace + "spin", spin_bridge->GetName());

    mBridgeName = pub_bridge->GetName();
    mTfBridgeName = tf_bridge->GetName();

    if (mConsole->mHasIO) {
        dvrk::add_topics_io(*pub_bridge, mNameSpace + "io", version);
    }

    const mtsIntuitiveResearchKitConsole::ArmList::iterator
        armEnd = mConsole->mArms.end();
    mtsIntuitiveResearchKitConsole::ArmList::iterator armIter;
    for (armIter = mConsole->mArms.begin();
         armIter != armEnd;
         ++armIter) {
        const std::string name = armIter->first;
        const std::string armNameSpace = mNameSpace + name;
        switch (armIter->second->mType) {
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_DERIVED:
            dvrk::add_tf_arm(*tf_bridge, name);
            dvrk::add_topics_mtm(*pub_bridge, armNameSpace, name, version);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_GENERIC:
            dvrk::add_topics_mtm_generic(*pub_bridge, armNameSpace, name, version);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED:
            dvrk::add_tf_arm(*tf_bridge, name);
            dvrk::add_topics_ecm(*pub_bridge, armNameSpace, name, version);
            if (armIter->second->mSimulation
                == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                dvrk::add_topics_ecm_io(*pub_bridge, armNameSpace,
                                        name, version);
            }
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED:
            dvrk::add_tf_arm(*tf_bridge, name);
            dvrk::add_topics_psm(*pub_bridge, armNameSpace, name, version);
            if (armIter->second->mSimulation
                == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                dvrk::add_topics_psm_io(*pub_bridge, armNameSpace,
                                        name, version);
            }
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ:
            dvrk::add_tf_suj(*tf_bridge, "PSM1");
            dvrk::add_tf_suj(*tf_bridge, "PSM2");
            dvrk::add_tf_suj(*tf_bridge, "PSM3");
            dvrk::add_tf_suj(*tf_bridge, "ECM");
            dvrk::add_topics_suj(*pub_bridge, mNameSpace + "SUJ/PSM1", "PSM1", version);
            dvrk::add_topics_suj(*pub_bridge, mNameSpace + "SUJ/PSM2", "PSM2", version);
            dvrk::add_topics_suj(*pub_bridge, mNameSpace + "SUJ/PSM3", "PSM3", version);
            dvrk::add_topics_suj(*pub_bridge, mNameSpace + "SUJ/ECM", "ECM", version);
        default:
            break;
        }
    }

    const mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator
        teleopsEnd = mConsole->mTeleopsPSM.end();
    mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator teleopIter;
    for (teleopIter = mConsole->mTeleopsPSM.begin();
         teleopIter != teleopsEnd;
         ++teleopIter) {
        const std::string name = teleopIter->first;
        std::string topic_name = teleopIter->first;
        std::replace(topic_name.begin(), topic_name.end(), '-', '_');
        dvrk::add_topics_teleop(*pub_bridge, mNameSpace + topic_name, name, version);
    }

    // digital inputs
    const std::string footPedalsNameSpace = mNameSpace + "footpedals/";
    typedef mtsIntuitiveResearchKitConsole::DInputSourceType DInputSourceType;
    const DInputSourceType::const_iterator inputsEnd = mConsole->mDInputSources.end();
    DInputSourceType::const_iterator inputsIter;
    for (inputsIter = mConsole->mDInputSources.begin();
         inputsIter != inputsEnd;
         ++inputsIter) {
        std::string upperName = inputsIter->second.second;
        std::string lowerName = inputsIter->first;
        // put everything lower case
        std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(), tolower);
        // replace +/- by strings
        cmnStringReplaceAll(lowerName, "-", "_minus");
        cmnStringReplaceAll(lowerName, "+", "_plus");
        pub_bridge->AddPublisherFromEventWrite<prmEventButton, sensor_msgs::Joy>
            (upperName + "_" + inputsIter->first, "Button",
             footPedalsNameSpace + lowerName);
    }

    dvrk::add_topics_console(*pub_bridge, mNameSpace + "console", version);
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
            mtsROSBridge * rosIOBridge = new mtsROSBridge(bridgeNamePrefix + name, period, true);
            dvrk::add_topics_io(*rosIOBridge,
                                mNameSpace + name + "/io/",
                                name, mVersion);
            componentManager->AddComponent(rosIOBridge);
            mIOInterfaces.push_back(name);
        }
    }
}

void dvrk::console::Connect(void)
{
    if (mConsole->mHasIO) {
        dvrk::connect_bridge_io(mBridgeName, mConsole->mIOComponentName);
    }

    const mtsIntuitiveResearchKitConsole::ArmList::iterator
        armEnd = mConsole->mArms.end();
    mtsIntuitiveResearchKitConsole::ArmList::iterator armIter;
    for (armIter = mConsole->mArms.begin();
         armIter != armEnd;
         ++armIter) {
        const std::string name = armIter->first;
        switch (armIter->second->mType) {
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_DERIVED:
            dvrk::connect_tf_arm(mTfBridgeName, name,
                                 armIter->second->ComponentName(),
                                 armIter->second->InterfaceName());
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_GENERIC:
            dvrk::connect_bridge_mtm(mBridgeName, name,
                                     armIter->second->ComponentName(),
                                     armIter->second->InterfaceName());
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED:
            dvrk::connect_tf_arm(mTfBridgeName, name,
                                 armIter->second->ComponentName(),
                                 armIter->second->InterfaceName());
            dvrk::connect_bridge_ecm(mBridgeName, name,
                                     armIter->second->ComponentName(),
                                     armIter->second->InterfaceName());
            if (armIter->second->mSimulation
                == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                dvrk::connect_bridge_ecm_io(mBridgeName, name,
                                            armIter->second->IOComponentName());
            }
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED:
            dvrk::connect_tf_arm(mTfBridgeName, name,
                                 armIter->second->ComponentName(),
                                 armIter->second->InterfaceName());
            dvrk::connect_bridge_psm(mBridgeName, name,
                                     armIter->second->ComponentName(),
                                     armIter->second->InterfaceName());
            if (armIter->second->mSimulation
                == mtsIntuitiveResearchKitConsole::Arm::SIMULATION_NONE) {
                dvrk::connect_bridge_psm_io(mBridgeName, name,
                                            armIter->second->IOComponentName());
            }
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ:
            dvrk::connect_tf_suj(mTfBridgeName, name, "PSM1");
            dvrk::connect_tf_suj(mTfBridgeName, name, "PSM2");
            dvrk::connect_tf_suj(mTfBridgeName, name, "PSM3");
            dvrk::connect_tf_suj(mTfBridgeName, name, "ECM");
            dvrk::connect_bridge_suj(mBridgeName, name, "PSM1");
            dvrk::connect_bridge_suj(mBridgeName, name, "PSM2");
            dvrk::connect_bridge_suj(mBridgeName, name, "PSM3");
            dvrk::connect_bridge_suj(mBridgeName, name, "ECM");
        default:
            break;
        }
    }

    const mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator
        teleopsEnd = mConsole->mTeleopsPSM.end();
    mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator teleopIter;
    for (teleopIter = mConsole->mTeleopsPSM.begin();
         teleopIter != teleopsEnd;
         ++teleopIter) {
        const std::string name = teleopIter->first;
        dvrk::connect_bridge_teleop(mBridgeName, name);
    }

    // connect foot pedal, all arms use same
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    typedef mtsIntuitiveResearchKitConsole::DInputSourceType DInputSourceType;
    const DInputSourceType::const_iterator inputsEnd = mConsole->mDInputSources.end();
    DInputSourceType::const_iterator inputsIter;
    for (inputsIter = mConsole->mDInputSources.begin();
         inputsIter != inputsEnd;
         ++inputsIter) {
        componentManager->Connect(mBridgeName, inputsIter->second.second + "_" + inputsIter->first,
                                  inputsIter->second.first, inputsIter->second.second);
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
