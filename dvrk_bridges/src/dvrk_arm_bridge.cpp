/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-01-19

  (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#include <dvrk_bridges/dvrk_arm_bridge.h>

dvrk_arm_bridge::dvrk_arm_bridge(const std::string & componentName,
                                 const std::string & rosNamespace,
                                 const double periodInSeconds,
                                 bool spin,
                                 bool sig,
                                 ros::NodeHandle * node_handle):
    mtsROSBridge(componentName, periodInSeconds, spin, sig, node_handle),
    ros_namespace(rosNamespace)
{
    setup();
}

void dvrk_arm_bridge::setup(void)
{
    // read
    this->AddPublisherFromReadCommand<prmPositionJointGet, cisst_msgs::vctDoubleVec>
        ("Robot", "GetPositionJoint", ros_namespace +"/joint_position_current");
    this->AddPublisherFromReadCommand<vctDoubleVec, cisst_msgs::vctDoubleVec>
        ("Robot", "GetPositionJointDesired", ros_namespace +"/position_joint_desired");
    this->AddPublisherFromReadCommand<prmPositionCartesianGet, geometry_msgs::Pose>
        ("Robot", "GetPositionCartesian", ros_namespace +"/position_cartesian_current");
    this->AddPublisherFromReadCommand<prmPositionCartesianGet, geometry_msgs::Pose>
        ("Robot", "GetPositionCartesianDesired", ros_namespace +"/position_cartesian_desired");

    // write
    this->AddSubscriberToWriteCommand<std::string, std_msgs::String>
        ("Robot", "SetRobotControlState", ros_namespace + "/set_robot_state");
    this->AddSubscriberToWriteCommand<prmPositionJointSet, cisst_msgs::vctDoubleVec>
        ("Robot", "SetPositionJoint", ros_namespace + "/set_position_joint");
    this->AddSubscriberToWriteCommand<prmPositionCartesianSet, geometry_msgs::Pose>
        ("Robot", "SetPositionCartesian", ros_namespace + "/set_position_cartesian");

    // events
    this->AddPublisherFromEventWrite<std::string, std_msgs::String>
        ("Robot","RobotState", ros_namespace + "/robot_state");
    this->AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        ("Robot","ManipClutch", ros_namespace + "/manip_clutch");
    this->AddPublisherFromEventWrite<prmEventButton, std_msgs::Bool>
        ("Robot","SUJClutch", ros_namespace + "/suj_clutch");
}
