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

// system
#include <iostream>
#include <map>

// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnGetChar.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQt.h>

#include <QApplication>
#include <QLocale>
#include <clocale>

#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <dvrk_utilities/dvrk_console.h>

void fileExists(const std::string & description, const std::string & filename)
{
    if (!cmnPath::Exists(filename)) {
        std::cerr << "File not found: " << description
                  << "; " << filename << std::endl;
        exit(-1);
    } else {
        std::cout << "File found: " << description
                  << "; " << filename << std::endl;
    }
}

int main(int argc, char ** argv)
{
    // replace the C++ global locale by C locale
    std::setlocale(LC_ALL, "C");

    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClass("mtsIntuitiveResearchKit", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // ---- WARNING: hack to remove ros args ----
    ros::V_string argout;
    ros::removeROSArgs(argc, argv, argout);
    argc = argout.size();
    // ------------------------------------------

    // parse options
    cmnCommandLineOptions options;
    std::string jsonMainConfigFile;
    std::string rosNamespace = "/dvrk/";
    double rosPeriod = 20.0 * cmn_ms;
    std::list<std::string> jsonIOConfigFiles;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonMainConfigFile);

    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all arms/teleop components and publish (default 0.02, 20 ms)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);

    options.AddOptionOneValue("n", "ros-namespace",
                              "ROS namespace to prefix all topics, must have start and end \"/\" (default /dvrk/)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosNamespace);

    options.AddOptionMultipleValues("i", "ros-io-config",
                                    "json config file to configure ROS bridges to collect low level data (IO)",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &jsonIOConfigFiles);

    options.AddOptionNoValue("t", "text-only",
                             "text only interface, do not create Qt widgets");

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    const bool hasQt = !options.IsSet("text-only");

    // console
    mtsIntuitiveResearchKitConsole * console = new mtsIntuitiveResearchKitConsole("console");
    fileExists("console JSON configuration file", jsonMainConfigFile);
    console->Configure(jsonMainConfigFile);
    componentManager->AddComponent(console);
    console->Connect();

    QApplication * application;
    mtsIntuitiveResearchKitConsoleQt * consoleQt = 0;
    // add all Qt widgets if needed
    if (hasQt) {
        QLocale::setDefault(QLocale::English);
        application = new QApplication(argc, argv);
        consoleQt = new mtsIntuitiveResearchKitConsoleQt();
        consoleQt->Configure(console);
        consoleQt->Connect();
    }

    // ros wrapper for arms and optionally IOs
    mtsROSBridge rosBridge("dVRKBridge", rosPeriod, true);
    dvrk::console * consoleROS = new dvrk::console(rosBridge, rosNamespace, console);
    // IOs
    const std::list<std::string>::const_iterator end = jsonIOConfigFiles.end();
    std::list<std::string>::const_iterator iter;
    for (iter = jsonIOConfigFiles.begin();
         iter != end;
         iter++) {
        fileExists("ROS IO JSON configuration file", *iter);
        consoleROS->Configure(*iter);
    }
    componentManager->AddComponent(&rosBridge);
    consoleROS->Connect();

    //-------------- create the components ------------------
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    if (hasQt) {
        application->exec();
    } else {
        do {
            std::cout << "Press 'q' to quit" << std::endl;
        } while (cmnGetChar() != 'q');
    }

    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // stop all logs
    cmnLogger::Kill();

    delete console;
    if (hasQt) {
        delete consoleQt;
    }
    delete consoleROS;

    return 0;
}
