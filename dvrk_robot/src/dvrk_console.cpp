/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-18

  (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <dvrk_utilities/dvrk_console.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>

#include <json/json.h>

const std::string bridgeNamePrefix = "dVRKIOBridge";

dvrk::console::console(mtsROSBridge & bridge,
                       const std::string & ros_namespace,
                       mtsIntuitiveResearchKitConsole * mts_console):
    mNameSpace(ros_namespace),
    mConsole(mts_console)
{
    mBridgeName = bridge.GetName();

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
            dvrk::add_topics_mtm(bridge, mNameSpace + "/" + name, name);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED:
            dvrk::add_topics_ecm(bridge, mNameSpace + "/" + name, name);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED:
            dvrk::add_topics_psm(bridge, mNameSpace + "/" + name, name);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ:
            dvrk::add_topics_suj(bridge, mNameSpace + "/SUJ/PSM1", "PSM1");
            dvrk::add_topics_suj(bridge, mNameSpace + "/SUJ/PSM2", "PSM2");
            dvrk::add_topics_suj(bridge, mNameSpace + "/SUJ/PSM3", "PSM3");
            dvrk::add_topics_suj(bridge, mNameSpace + "/SUJ/ECM", "ECM");
        default:
            break;
        }
    }

    const mtsIntuitiveResearchKitConsole::TeleopList::iterator
        teleopsEnd = mConsole->mTeleops.end();
    mtsIntuitiveResearchKitConsole::TeleopList::iterator teleopIter;
    for (teleopIter = mConsole->mTeleops.begin();
         teleopIter != teleopsEnd;
         ++teleopIter) {
        const std::string name = teleopIter->first;
        std::string topic_name = teleopIter->first;
        std::replace(topic_name.begin(), topic_name.end(), '-', '_');
        dvrk::add_topics_teleop(bridge, mNameSpace + "/" + topic_name, name);
    }

    if (mConsole->mHasFootpedals) {
        dvrk::add_topics_footpedals(bridge, mNameSpace + "/footpedals");
    }
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
                                name);
            componentManager->AddComponent(rosIOBridge);
            mIOInterfaces.push_back(name);
        }
    }
}

void dvrk::console::Connect(void)
{
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
            dvrk::connect_bridge_mtm(mBridgeName, name);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED:
            dvrk::connect_bridge_ecm(mBridgeName, name);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED:
            dvrk::connect_bridge_psm(mBridgeName, name);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ:
            dvrk::connect_bridge_suj(mBridgeName, name, "PSM1");
            dvrk::connect_bridge_suj(mBridgeName, name, "PSM2");
            dvrk::connect_bridge_suj(mBridgeName, name, "PSM3");
            dvrk::connect_bridge_suj(mBridgeName, name, "ECM");
        default:
            break;
        }
    }

    const mtsIntuitiveResearchKitConsole::TeleopList::iterator
        teleopsEnd = mConsole->mTeleops.end();
    mtsIntuitiveResearchKitConsole::TeleopList::iterator teleopIter;
    for (teleopIter = mConsole->mTeleops.begin();
         teleopIter != teleopsEnd;
         ++teleopIter) {
        const std::string name = teleopIter->first;
        dvrk::connect_bridge_teleop(mBridgeName, name);
    }

    // connect foot pedal, all arms use same
    if (mConsole->mHasFootpedals) {
        dvrk::connect_bridge_footpedals(mBridgeName, mConsole->mIOComponentName);
    }

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
