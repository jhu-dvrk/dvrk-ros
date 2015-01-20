/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-01-20

  (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <iostream>

#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnCommandLineOptions.h>

#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsQtApplication.h>

#include <sawControllers/mtsPID.h>
#include <sawControllers/mtsPIDQtWidget.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>

#include <dvrk_bridges/dvrk_arm_bridge.h>


int main(int argc, char** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // ---- remove ros args ----
    ros::V_string argout;
    ros::removeROSArgs(argc, argv, argout);
    argc = argout.size();

    // parse options
    int firewirePort = 0;
    std::string config_io;
    std::string config_pid;
    std::string config_kinematics;
    std::string config_type;
    cmnCommandLineOptions options;
    options.AddOptionOneValue("i", "io", "config file for robot IO",
                              cmnCommandLineOptions::REQUIRED_OPTION, &config_io);
    options.AddOptionOneValue("p", "pid", "config file for PID controller",
                              cmnCommandLineOptions::REQUIRED_OPTION, &config_pid);
    options.AddOptionOneValue("k", "kinematic", "config file for robot kinematic",
                              cmnCommandLineOptions::REQUIRED_OPTION, &config_kinematics);
    options.AddOptionOneValue("t", "type", "robot type (must be either ECM, MTM or PSM",
                              cmnCommandLineOptions::REQUIRED_OPTION, &config_type);
    options.AddOptionOneValue("f", "firewire", "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &firewirePort);

    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    // type of arm
    mtsComponent * arm = 0;
    std::string arm_name = config_type;
    if (config_type == "ECM") {
        arm = new mtsIntuitiveResearchKitECM(arm_name, 5.0 * cmn_ms);
    } else if (config_type == "MTM") {
        arm = new mtsIntuitiveResearchKitMTM(arm_name, 5.0 * cmn_ms);
    } else if (config_type == "PSM") {
        arm = new mtsIntuitiveResearchKitPSM(arm_name, 5.0 * cmn_ms);
    } else {
        std::cerr << "Error: arm type must be either ECM, MTM or PSM, not \"" << config_type << "\"" << std::endl;
        return -1;
    }

    // now components
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // create a Qt application and tab to hold all widgets
    mtsQtApplication * qtAppTask = new mtsQtApplication("QtApplication", argc, argv);
    qtAppTask->Configure();
    componentManager->AddComponent(qtAppTask);

    // IO
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", 1.0 * cmn_ms, firewirePort);
    io->Configure(config_io);
    componentManager->AddComponent(io);

    // connect ioGUI to io
    mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
    componentManager->AddComponent(robotWidgetFactory);
    componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "io", "Configuration");
    robotWidgetFactory->Configure();

    // mtsPID
    mtsPID * pid = new mtsPID("pid", 5 * cmn_ms);
    pid->Configure(config_pid);
    componentManager->AddComponent(pid);
    componentManager->Connect(pid->GetName(), "RobotJointTorqueInterface", "io", arm_name);
    componentManager->Connect(pid->GetName(), "ExecIn", "io", "ExecOut"); // PID will run in IO thread

    // PID  GUI
    mtsPIDQtWidget * pidGUI = new mtsPIDQtWidget("pid master",8);
    pidGUI->Configure();
    componentManager->AddComponent(pidGUI);
    componentManager->Connect(pidGUI->GetName(), "Controller", pid->GetName(), "Controller");

    // arm
    arm->Configure(config_kinematics);
    componentManager->AddComponent(arm);
    componentManager->Connect(arm->GetName(), "PID", pid->GetName(), "Controller");
    componentManager->Connect(arm->GetName(), "RobotIO", "io", arm_name);

    // arm GUI
    mtsIntuitiveResearchKitArmQtWidget* armGUI = new mtsIntuitiveResearchKitArmQtWidget(arm_name + "GUI");
    componentManager->AddComponent(armGUI);
    componentManager->Connect(armGUI->GetName(), "Manipulator", arm->GetName(), "Robot");

    // add ROS bridge
    dvrk_arm_bridge * armROS = new dvrk_arm_bridge(arm_name + "ROS", // mts component name
                                                   "/dvrk/" + arm_name, // ROS namespace
                                                   10.0 * cmn_ms,
                                                   true); // this component performs ros::spin
    componentManager->AddComponent(armROS);
    componentManager->Connect(armROS->GetName(), "Robot", arm->GetName(), "Robot");

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;
    // io gui
    mtsRobotIO1394QtWidgetFactory::WidgetListType::const_iterator iterator;
    for (iterator = robotWidgetFactory->Widgets().begin();
         iterator != robotWidgetFactory->Widgets().end();
         ++iterator) {
            tabWidget->addTab(*iterator, (*iterator)->GetName().c_str());
        }
    // pid gui
    tabWidget->addTab(pidGUI, (arm_name + "PID").c_str());
    // arm gui
    tabWidget->addTab(armGUI, arm->GetName().c_str());
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

    // QtApp will run here
    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // delete dvgc robot
    delete io;
    delete pid;
    delete pidGUI;
    delete arm;
    delete armGUI;
    delete armROS;

    // stop all logs
    cmnLogger::Kill();
}
