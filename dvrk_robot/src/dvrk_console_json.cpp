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

// system
#include <iostream>
#include <map>

// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnQt.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQt.h>

#include <QApplication>
#include <QIcon>
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
    cmnLogger::SetMaskClassMatching("mtsIntuitiveResearchKit", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);
    // add log file with date so logs don't get overwritten
    std::string currentDateTime;
    osaGetDateTimeString(currentDateTime);
    std::ofstream logFileStream(std::string("cisstLog-" + currentDateTime + ".txt").c_str());
    cmnLogger::AddChannel(logFileStream);
    cmnLogger::HaltDefaultLog(); // stop log to default cisstLog.txt

    // ---- WARNING: hack to remove ros args ----
    ros::V_string argout;
    ros::removeROSArgs(argc, argv, argout);
    argc = argout.size();
    // ------------------------------------------

    // parse options
    cmnCommandLineOptions options;
    std::string jsonMainConfigFile;
    std::string rosNamespace = "dvrk/";
    double rosPeriod = 10.0 * cmn_ms;
    double tfPeriod = 20.0 * cmn_ms;
    std::list<std::string> jsonIOConfigFiles;
    std::string versionString = "v1_4_0";
    typedef std::list<std::string> managerConfigType;
    managerConfigType managerConfig;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonMainConfigFile);

    options.AddOptionOneValue("n", "ros-namespace",
                              "ROS namespace to prefix all topics, must end with \"/\" if not empty (default is \"dvrk/\")",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosNamespace);

    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all arms/teleop components and publish (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the arm component's period",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);

    options.AddOptionOneValue("P", "tf-ros-period",
                              "period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the arm component's period",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &tfPeriod);

    options.AddOptionMultipleValues("i", "ros-io-config",
                                    "json config file to configure ROS bridges to collect low level data (IO)",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &jsonIOConfigFiles);

    options.AddOptionNoValue("t", "text-only",
                             "text only interface, do not create Qt widgets");

    options.AddOptionOneValue("c", "compatibility",
                              "compatibility mode, e.g. \"v1_3_0\", \"v1_4_0\"",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &versionString);

    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments;

    // check version mode
    dvrk_topics_version::version versionEnum;
    try {
        versionEnum = dvrk_topics_version::versionFromString(versionString);
    } catch (std::exception e) {
        std::cerr << "Compatibility mode " << versionString << " is invalid" << std::endl;
        std::cerr << "Possible values are: ";
        std::cerr << cmnData<std::vector<std::string> >::HumanReadable(dvrk_topics_version::versionVectorString());
        std::cerr << std::endl;
        return -1;
    }
    std::cout << "Using compatibility mode: " << versionString << std::endl;

    const bool hasQt = !options.IsSet("text-only");

    // start creating components
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

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
        application->setWindowIcon(QIcon(":/dVRK.png"));
        cmnQt::QApplicationExitsOnCtrlC();
        consoleQt = new mtsIntuitiveResearchKitConsoleQt();
        consoleQt->Configure(console);
        consoleQt->Connect();
    }

    // create a console with all dVRK ROS topics
    // - rosPeriod is used to control publish rate
    // - tfPeriod is used to control tf broadcast rate
    //
    // this also adds a mtsROSBridge that performs the ros::spinOnce
    // in a separate thread as fast possible
    dvrk::console * consoleROS = new dvrk::console(rosPeriod, tfPeriod, rosNamespace,
                                                   console, versionEnum);
    // IOs
    const std::list<std::string>::const_iterator end = jsonIOConfigFiles.end();
    std::list<std::string>::const_iterator iter;
    for (iter = jsonIOConfigFiles.begin();
         iter != end;
         iter++) {
        fileExists("ROS IO JSON configuration file", *iter);
        consoleROS->Configure(*iter);
    }

    consoleROS->Connect();

    // custom user component
    const managerConfigType::iterator endConfig = managerConfig.end();
    for (managerConfigType::iterator iterConfig = managerConfig.begin();
         iterConfig != endConfig;
         ++iterConfig) {
        if (!iterConfig->empty()) {
            if (!cmnPath::Exists(*iterConfig)) {
                CMN_LOG_INIT_ERROR << "File " << *iterConfig
                                   << " not found!" << std::endl;
            } else {
                if (!componentManager->ConfigureJSON(*iterConfig)) {
                    CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager for "
                                       << *iterConfig << std::endl;
                    return -1;
                }
            }
        }
    }

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
    cmnLogger::RemoveChannel(logFileStream);

    delete console;
    if (hasQt) {
        delete consoleQt;
    }
    delete consoleROS;

    return 0;
}
