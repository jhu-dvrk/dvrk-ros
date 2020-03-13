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

#include <dvrk_psm_from_ros.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

#include <cmath>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(dvrk_psm_from_ros,
                                      dvrk_arm_from_ros,
                                      mtsTaskPeriodicConstructorArg);

dvrk_psm_from_ros::dvrk_psm_from_ros(const std::string & componentName,
                                     const double periodInSeconds)
    : dvrk_arm_from_ros(componentName, periodInSeconds)
{
    InitPSM();
}

dvrk_psm_from_ros::dvrk_psm_from_ros(const mtsTaskPeriodicConstructorArg & arg)
    : dvrk_arm_from_ros(arg.Name, arg.Period)
{
    InitPSM();
}

void dvrk_psm_from_ros::InitPSM(void)
{
    std::string ros_namespace = this->GetName();
    std::string interface_provided = this->GetName();

    AddPublisherFromCommandWrite<prmPositionJointSet, sensor_msgs::JointState>
        (interface_provided,
         "SetPositionJaw",
         ros_namespace + "/set_position_jaw");

    AddSubscriberToCommandRead<prmStateJoint, sensor_msgs::JointState>
        (interface_provided,
         "GetStateJaw",
         ros_namespace + "/state_jaw_current");

    AddSubscriberToCommandRead<prmStateJoint, sensor_msgs::JointState>
        (interface_provided,
         "GetStateJawDesired",
         ros_namespace + "/state_jaw_desired");
}
