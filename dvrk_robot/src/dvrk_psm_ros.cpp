/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*
  $Id: mainQtPSM.cpp 4588 2013-12-04 22:53:51Z adeguet1 $

  Author(s):  Adnan Munawar (WPI), Modified from original file from Zihan Chen
  Created on: 2014-03-07

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#include <iostream>
#include <cisstCommon.h>
#include <cisstVector.h>
#include <cisstMultiTaskQt.h>

#include <sawControllers/mtsPID.h>
#include <sawControllers/mtsPIDQtWidget.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>

#include <cisst_ros_bridge/mtsROSBridge.h>


int main(int argc, char** argv)
{
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
  mtsQtApplication *qtAppTask = new mtsQtApplication("QtApplication", argc, argv);
  qtAppTask->Configure();
  componentManager->AddComponent(qtAppTask);

  // IO
  mtsRobotIO1394 * io = new mtsRobotIO1394("io", 1.0 * cmn_ms, firewirePort);
  io->Configure(config_io);
  componentManager->AddComponent(io);

  // connect ioGUIMaster to io
  mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
  componentManager->AddComponent(robotWidgetFactory);
  componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "io", "Configuration");
  robotWidgetFactory->Configure();

  // mtsPID
  mtsPID* pid = new mtsPID("pid", 5 * cmn_ms);
  pid->Configure(config_pid);
  componentManager->AddComponent(pid);
  componentManager->Connect(pid->GetName(), "RobotJointTorqueInterface", "io", config_name);
  componentManager->Connect(pid->GetName(), "ExecIn", "io", "ExecOut");

  // PID Master GUI
  mtsPIDQtWidget * pidMasterGUI = new mtsPIDQtWidget("pid master",7);
  pidMasterGUI->Configure();
  componentManager->AddComponent(pidMasterGUI);
  componentManager->Connect(pidMasterGUI->GetName(), "Controller", pid->GetName(), "Controller");

  // psm
  mtsIntuitiveResearchKitPSM* psm = new mtsIntuitiveResearchKitPSM(config_name, 5 * cmn_ms);
  psm->Configure(config_kinematics);
  componentManager->AddComponent(psm);
  componentManager->Connect(psm->GetName(), "PID", pid->GetName(), "Controller");
  componentManager->Connect(psm->GetName(), "RobotIO", "io", config_name);

  // psm GUI
  mtsIntuitiveResearchKitArmQtWidget* psmGUI = new mtsIntuitiveResearchKitArmQtWidget(config_name+"GUI");
  componentManager->AddComponent(psmGUI);
  componentManager->Connect(psmGUI->GetName(), "Manipulator", psm->GetName(), "Robot");


  //-------------------------------------------------------
  // Start ROS Bridge
  // ------------------------------------------------------
  ROS_INFO("\n Name is :%s \n :%s \n :%s \n :%s \n ",config_name.c_str(),config_pid.c_str(),config_io.c_str()
           ,config_kinematics.c_str());
  // ros wrapper
  mtsROSBridge robotBridge("RobotBridge", 20 * cmn_ms, true);
  // connect to psm

  robotBridge.AddPublisherFromReadCommand<prmPositionCartesianGet, geometry_msgs::Pose>(
        config_name, "GetPositionCartesian", "/dvrk_psm/cartesian_pose_current");
  robotBridge.AddPublisherFromReadCommand<vctDoubleVec, cisst_msgs::vctDoubleVec>(
        pid->GetName(), "GetEffortJoint", "/dvrk_psm/joint_effort_current");
  robotBridge.AddPublisherFromReadCommand<prmPositionJointGet, sensor_msgs::JointState>(
        pid->GetName(), "GetPositionJoint", "/dvrk_psm/joint_position_current");


  // Finally Working Form; However it is still unsafe since there is no safety check.
  // Use with caution and with your hand on the E-Stop.
  robotBridge.AddSubscriberToWriteCommand<prmForceTorqueJointSet , sensor_msgs::JointState>(
        pid->GetName(), "SetTorqueJoint", "/dvrk_psm/set_joint_effort");
  robotBridge.AddSubscriberToWriteCommand<std::string, std_msgs::String>(
        config_name, "SetRobotControlState", "/dvrk_psm/set_robot_state");
  robotBridge.AddSubscriberToWriteCommand<bool, std_msgs::Bool>(
        pid->GetName(),"Enable","/dvrk_psm/enable_pid");
  robotBridge.AddSubscriberToWriteCommand<prmPositionJointSet, sensor_msgs::JointState>(
        pid->GetName(),"SetPositionJoint","/dvrk_psm/set_position_joint");
  robotBridge.AddSubscriberToWriteCommand<prmPositionCartesianSet, geometry_msgs::Pose>(
        config_name, "SetPositionCartesian", "/dvrk_psm/set_position_cartesian");

  componentManager->AddComponent(&robotBridge);
  componentManager->Connect(robotBridge.GetName(), config_name, psm->GetName(), "Robot");
  componentManager->Connect(robotBridge.GetName(), pid->GetName(), pid->GetName(),"Controller");

//  componentManager->Connect(robotBridge.GetName(), "Clutch", "io", "CLUTCH");

  //-------------------------------------------------------
  // End ROS Bridge
  // ------------------------------------------------------


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
  tabWidget->addTab(pidMasterGUI, (config_name + "PID").c_str());
  // psm gui
  tabWidget->addTab(psmGUI, psm->GetName().c_str());
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
  delete pid;
  delete pidMasterGUI;
  delete psm;
  delete psmGUI;

  // stop all logs
  cmnLogger::Kill();
}







