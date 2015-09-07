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

dvrk::console::console(mtsROSBridge & bridge,
                       const std::string & ros_namespace,
                       mtsIntuitiveResearchKitConsole * mts_console):
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
            dvrk::add_topics_mtm(bridge, ros_namespace + "/" + name, name);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
            dvrk::add_topics_ecm(bridge, ros_namespace + "/" + name, name);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
            dvrk::add_topics_psm(bridge, ros_namespace + "/" + name, name);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ:
            dvrk::add_topics_suj(bridge, ros_namespace + "/SUJ/PSM1", "PSM1");
            dvrk::add_topics_suj(bridge, ros_namespace + "/SUJ/PSM2", "PSM2");
            dvrk::add_topics_suj(bridge, ros_namespace + "/SUJ/PSM3", "PSM3");
            dvrk::add_topics_suj(bridge, ros_namespace + "/SUJ/ECM", "ECM");
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
        dvrk::add_topics_teleop(bridge, ros_namespace + "/" + topic_name, name);
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
            dvrk::connect_bridge_mtm(mBridgeName, name);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
            dvrk::connect_bridge_ecm(mBridgeName, name);
            break;
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
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
}
