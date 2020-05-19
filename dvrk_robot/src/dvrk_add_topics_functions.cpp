/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2015-04-33

  (C) Copyright 2015-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisst_ros_crtk/mtsCISSTToROS.h>
#include <cisst_ros_crtk/mtsROSToCISST.h>

#include <dvrk_utilities/dvrk_add_topics_functions.h>

void dvrk::add_topics_suj(mtsROSBridge & bridge,
                          const std::string & ros_namespace,
                          const std::string & arm_name)
{
    // events
    bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        (arm_name + "-suj", "measured_js",
         ros_namespace + "/measured_js");
    bridge.AddSubscriberToCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
        (arm_name + "-suj", "servo_jp",
         ros_namespace + "/servo_jp");
    bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
        (arm_name + "-suj", "measured_cp_local",
         ros_namespace + "/local/measured_cp");
    bridge.AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::TransformStamped>
        (arm_name + "-suj", "measured_cp",
         ros_namespace + "/measured_cp");

    // messages
    bridge.AddLogFromEventWrite(arm_name + "-suj-log", "error",
                                mtsROSEventWriteLog::ROS_LOG_ERROR);
    bridge.AddLogFromEventWrite(arm_name + "-suj-log", "warning",
                                mtsROSEventWriteLog::ROS_LOG_WARN);
    bridge.AddLogFromEventWrite(arm_name + "-suj-log", "status",
                                mtsROSEventWriteLog::ROS_LOG_INFO);
}

void dvrk::connect_bridge_suj(const std::string & bridge_name,
                              const std::string & suj_component_name,
                              const std::string & arm_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, arm_name + "-suj",
                              suj_component_name, arm_name);
    componentManager->Connect(bridge_name, arm_name + "-suj-log",
                              suj_component_name, arm_name);
}

void dvrk::add_topics_io(mtsROSBridge & bridge,
                         const std::string & ros_namespace)
{
    bridge.AddPublisherFromCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        ("io", "period_statistics",
         ros_namespace + "/period_statistics");
    bridge.AddPublisherFromCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        ("io", "GetPeriodStatisticsRead",
         ros_namespace + "/period_statistics_read");
    bridge.AddPublisherFromCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        ("io", "GetPeriodStatisticsWrite",
         ros_namespace + "/period_statistics_write");
}

void dvrk::connect_bridge_io(const std::string & bridge_name,
                             const std::string & io_component_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, "io",
                              io_component_name, "Configuration");
}

void dvrk::add_topics_io(mtsROSBridge & bridge,
                         const std::string & ros_namespace,
                         const std::string & arm_name)
{
    bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (arm_name + "-io", "GetAnalogInputPosSI",
         ros_namespace + "/analog_input_pos_si");
    bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        (arm_name + "-io", "measured_js",
         ros_namespace + "/joint_measured_js");
    bridge.AddPublisherFromCommandRead<prmStateJoint, sensor_msgs::JointState>
        (arm_name + "-io", "actuator_measured_js",
         ros_namespace + "/actuator_measured_js");
    bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (arm_name + "-io", "GetActuatorFeedbackCurrent",
         ros_namespace + "/actuator_current_measured");
    bridge.AddPublisherFromCommandRead<vctDoubleVec, sensor_msgs::JointState>
        (arm_name + "-io", "GetActuatorRequestedCurrent",
         ros_namespace + "/actuator_current_requested");
}

void dvrk::connect_bridge_io(const std::string & bridge_name,
                             const std::string & io_component_name,
                             const std::string & arm_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(bridge_name, arm_name + "-io",
                              io_component_name, arm_name);
}

void dvrk::add_tf_suj(mtsROSBridge & tf_bridge,
                      const std::string & arm_name)
{
    tf_bridge.Addtf2BroadcasterFromCommandRead(arm_name + "-suj", "measured_cp");
}

void dvrk::connect_tf_suj(const std::string & tf_bridge_name,
                          const std::string & suj_component_name,
                          const std::string & arm_name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(tf_bridge_name, arm_name + "-suj",
                              suj_component_name, arm_name);
}
