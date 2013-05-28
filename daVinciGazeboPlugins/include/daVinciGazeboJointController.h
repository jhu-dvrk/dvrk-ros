/*
*  Copyright (c) 2013, Nirav Patel, AIM Lab, Worcester Polytechnic Institute
*  All rights reserved.
*
*  This software is provided "as is" under an open source license, with
*  no warranty. 
*
*  daVinciGazeboJointController.h
*
*  Created on: Feb 2, 2013
*  Author: Nirav Patel
*/

#ifndef DAVINCI_GAZEBO_JOINT_CONTROLLER_HH
#define DAVINCI_GAZEBO_JOINT_CONTROLLER_HH

#include <vector>

#include <sdf/interface/Param.hh>
#include <sdf/interface/SDF.hh>
#include <urdf/model.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "physics/physics.h"
#include "transport/TransportTypes.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"
#include "physics/PhysicsTypes.hh" 
#include "sensors/SensorTypes.hh"
#include "physics/physics.hh" 
#include <common/common.hh>

#include "boost/thread/mutex.hpp"
#include <ros/ros.h>
//#include <control_toolbox/pid.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include "daVinciGazeboPlugins/SetJointState.h"


namespace gazebo
{
class daVinciGazeboJointController : public ModelPlugin
{
  public:
  //Constructor  
   daVinciGazeboJointController();
  //Destrutor
  virtual ~daVinciGazeboJointController();
  
  protected:
  //Load the gazebo controller
  public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
  virtual void Init();

  //Method to be executed every update cycle
  private:virtual void OnUpdate();

  //function to reset all joints to zero position
  void InitializeJointControllers();
 
  private:
  event::ConnectionPtr updateConnection;
  
  // for setting ROS name space and robot description
  std::string robot_namespace_;
  std::string robot_description_parameter_;

  physics::JointController *jc;

  //map of jointName<String>->Joint positions<double>
  std::map<std::string, double> joint_positions;

  // parent Model
  physics::ModelPtr myParent;

  // ROS Nodehandle
  ros::NodeHandle* node;

  // ROS Subscriber for JointState messages
  ros::Subscriber sub;
 //callback functing for JointState which are received by sub ROS subscriber
  void ROSCallback(const sensor_msgs::JointState::ConstPtr& msg);

  // joint state publishes publishing JointState messages
  ros::Publisher joint_state_publisher;
  //function publishing joint stses
  void PublishJointStates();

  //service for setting joint states on call
   ros::ServiceServer SetJointStateService; 
  //fuction which is called on service call to SetJointStateService 
  bool UpdateJointStates(daVinciGazeboPlugins::SetJointState::Request  &req,daVinciGazeboPlugins::SetJointState::Response &res);

 };

}

#endif

