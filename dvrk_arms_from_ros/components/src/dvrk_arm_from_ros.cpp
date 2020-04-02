/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-01-13

  (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <dvrk_arm_from_ros.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

#include <cmath>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(dvrk_arm_from_ros,
                                      mtsROSBridge,
                                      mtsTaskPeriodicConstructorArg);

dvrk_arm_from_ros::dvrk_arm_from_ros(const std::string & componentName,
                                     const double periodInSeconds)
    : mtsROSBridge(componentName, periodInSeconds)
{
    Init();
}

dvrk_arm_from_ros::dvrk_arm_from_ros(const mtsTaskPeriodicConstructorArg & arg)
    : mtsROSBridge(arg.Name, arg.Period)
{
    Init();
}

void dvrk_arm_from_ros::Init(void)
{

    std::string ros_namespace = this->GetName();
    std::string interface_provided = this->GetName();

    AddPublisherFromCommandWrite<std::string, std_msgs::String>
        (interface_provided,
         "SetDesiredState",
         ros_namespace + "/set_desired_state");

    AddPublisherFromCommandVoid
        (interface_provided,
         "Freeze",
         ros_namespace + "/freeze");

    AddPublisherFromCommandWrite<prmPositionCartesianSet, geometry_msgs::Pose>
        (interface_provided,
         "servo_cp",
         ros_namespace + "/set_position_cartesian");

    AddPublisherFromCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
        (interface_provided,
         "SetPositionJaw",
         ros_namespace + "/set_position_jaw");

    AddSubscriberToCommandRead<mtsStdString, std_msgs::String>
        (interface_provided,
         "GetCurrentState",
         ros_namespace + "/current_state");

    AddSubscriberToCommandRead<mtsStdString, std_msgs::String>
        (interface_provided,
         "GetDesiredState",
         ros_namespace + "/desired_state");

    AddSubscriberToCommandRead<prmStateJoint, sensor_msgs::JointState>
        (interface_provided,
         "setpoint_js",
         ros_namespace + "/state_joint_desired");

    AddSubscriberToCommandRead<prmStateJoint, sensor_msgs::JointState>
        (interface_provided,
         "GetStateJaw",
         ros_namespace + "/state_jaw_current");

    AddSubscriberToCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
        (interface_provided,
         "measured_cp",
         ros_namespace + "/position_cartesian_current");

    AddSubscriberToCommandRead<mtsIntervalStatistics, cisst_msgs::mtsIntervalStatistics>
        (interface_provided,
         "period_statistics",
         ros_namespace + "/period_statistics");

    AddSubscriberToEventWrite<mtsMessage, std_msgs::String>
        (interface_provided, "Error",
         ros_namespace + "/error");

    AddSubscriberToEventWrite<mtsMessage, std_msgs::String>
        (interface_provided, "Warning",
         ros_namespace + "/warning");
    
    AddSubscriberToEventWrite<mtsMessage, std_msgs::String>
        (interface_provided, "Status",
         ros_namespace + "/status");
}

// Configure is a virtual method, we can redefine it and have our own
// configuration
void dvrk_arm_from_ros::Configure(const std::string & CMN_UNUSED(filename))
{

}
