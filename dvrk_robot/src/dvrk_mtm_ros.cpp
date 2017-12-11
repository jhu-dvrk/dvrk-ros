/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen
  Created on: 2013-07-15

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <iostream>

#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstMultiTask/mtsQtApplication.h>

#include <sawControllers/mtsPID.h>
#include <sawControllers/mtsPIDQtWidget.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>

#include <cisst_ros_bridge/mtsROSBridge.h>
#include <dvrk_utilities/dvrk_add_topics_functions.h>

int main(int argc, char** argv)
{
    // program deprecated
    std::cout << "-----------------------------------------------------------" << std::endl
              << "- This program is deprecated:                             -" << std::endl
              << "-   use dvrk_console_json instead                         -" << std::endl
              << "-   examples can be found in share/jhu-dVRK/console*.json -" << std::endl
              << "-----------------------------------------------------------" << std::endl
              << std::endl;

    // desired frequencies
    const double ioPeriod = 0.5 * cmn_ms;
    const double armPeriod = 2.0 * cmn_ms;
    const double rosPeriod = 10.0 * cmn_ms;

    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // ---- WARNING: hack to remove ros args ----
    ros::V_string argout;
    ros::removeROSArgs(argc, argv, argout);
    argc = argout.size();
    // ------------------------------------------

    // parse options
    int firewirePort = 0;
    std::string config_io;
    std::string config_pid;
    std::string config_kinematics;
    std::string config_name;
    cmnCommandLineOptions options;
    options.AddOptionOneValue("i", "io-master", "config file for master robot IO",
                              cmnCommandLineOptions::REQUIRED_OPTION, &config_io);
    options.AddOptionOneValue("p", "pid-master", "config file for master PID controller",
                              cmnCommandLineOptions::REQUIRED_OPTION, &config_pid);
    options.AddOptionOneValue("k", "kinematic-master", "config file for master robot kinematic",
                              cmnCommandLineOptions::REQUIRED_OPTION, &config_kinematics);
    options.AddOptionOneValue("n", "name-master", "config file for master robot name",
                              cmnCommandLineOptions::REQUIRED_OPTION, &config_name);
    options.AddOptionOneValue("f", "firewire", "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &firewirePort);

    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    // now components
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // create a Qt application and tab to hold all widgets
    mtsQtApplication * qtAppTask = new mtsQtApplication("QtApplication", argc, argv);
    qtAppTask->Configure();
    componentManager->AddComponent(qtAppTask);

    // IO
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", ioPeriod, firewirePort);
    io->Configure(config_io);
    componentManager->AddComponent(io);

    // connect ioGUIMaster to io
    mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
    componentManager->AddComponent(robotWidgetFactory);
    componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "io", "Configuration");
    robotWidgetFactory->Configure();

    // mtsPID
    mtsPID * pid = new mtsPID("pid", ioPeriod); // periodicity will be ignore because ExecIn/Out are connected
    pid->Configure(config_pid);
    componentManager->AddComponent(pid);
    componentManager->Connect(pid->GetName(), "RobotJointTorqueInterface", "io", config_name);
    componentManager->Connect(pid->GetName(), "ExecIn", "io", "ExecOut");

    // PID GUI
    mtsPIDQtWidget * pidMasterGUI = new mtsPIDQtWidget("pid master", 8);
    pidMasterGUI->Configure();
    componentManager->AddComponent(pidMasterGUI);
    componentManager->Connect(pidMasterGUI->GetName(), "Controller", pid->GetName(), "Controller");

    // mtm
    mtsIntuitiveResearchKitMTM * mtm = new mtsIntuitiveResearchKitMTM(config_name, armPeriod);
    mtm->Configure(config_kinematics);
    componentManager->AddComponent(mtm);
    componentManager->Connect(mtm->GetName(), "PID", pid->GetName(), "Controller");
    componentManager->Connect(mtm->GetName(), "RobotIO", "io", config_name);

    // mtm GUI
    mtsIntuitiveResearchKitArmQtWidget * mtmGUI = new mtsIntuitiveResearchKitArmQtWidget(config_name + "GUI");
    componentManager->AddComponent(mtmGUI);
    componentManager->Connect(mtmGUI->GetName(), "Manipulator", mtm->GetName(), "Robot");

    // ros wrapper
    mtsROSBridge robotBridge("RobotBridge", rosPeriod, true);

    // populate interfaces
    const dvrk_topics_version::version version = dvrk_topics_version::v1_3_0;
    dvrk::add_topics_mtm(robotBridge, "/dvrk/" + config_name, mtm->GetName(), version);
    dvrk::add_topics_footpedals(robotBridge, "/dvrk/" + config_name + "/footpedals", version);

    // add component
    componentManager->AddComponent(&robotBridge);

    // connect interfaces on cisst/SAW side
    dvrk::connect_bridge_mtm(robotBridge.GetName(), mtm->GetName(),
                             mtm->GetName(), "Robot");
    dvrk::connect_bridge_footpedals(robotBridge.GetName(), io->GetName());

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;
    // io gui
    mtsRobotIO1394QtWidgetFactory::WidgetListType::const_iterator iterator;
    for (iterator = robotWidgetFactory->Widgets().begin();
         iterator != robotWidgetFactory->Widgets().end();
         ++iterator)
        {
            tabWidget->addTab(*iterator, (*iterator)->GetName().c_str());
        }
    // pid gui
    tabWidget->addTab(pidMasterGUI, (config_name + " PID").c_str());
    // mtm gui
    tabWidget->addTab(mtmGUI, mtm->GetName().c_str());
    // button gui
    tabWidget->addTab(robotWidgetFactory->ButtonsWidget(), "Buttons");
    // show widget
    tabWidget->show();


    //-------------- create the components ------------------
    io->CreateAndWait(2.0 * cmn_s); // this will also create the pids as they are in same thread
    io->StartAndWait(2.0 * cmn_s);
    pid->StartAndWait(2.0 * cmn_s);

    // start all other components
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    // QtApp is now running

    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // delete components
    delete pidMasterGUI;
    delete pid;
    delete mtmGUI;
    delete mtm;

    // stop all logs
    cmnLogger::Kill();
}
