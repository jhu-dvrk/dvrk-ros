/*
*  Copyright (c) 2014, Adnan Munawar, AIM Lab, Worcester Polytechnic Institute
*  All rights reserved.
*
*  This software is provided "as is" under an open source license, with
*  no warranty. 
*
*  dvrk_gazebo_controller.h
*
*  Created on: April 27, 2014
*  Author: Adnan Munawar
*  Old Support :Nirav Patel
*/

#ifndef DVRK_GAZEBO_CONTROLLER_HH
#define DVRK_GAZEBO_CONTROLLER_HH

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <gazebo.hh>
#include <physics/physics.hh>
#include <stdio.h>
#include "dv_gazebo_plugins/SetJointState.h"


namespace gazebo
{
class dvrk_gazebo_controller : public ModelPlugin
{
  public:
  //Constructor  
   dvrk_gazebo_controller();
  //Destrutor
  virtual ~dvrk_gazebo_controller();
  
  protected:
  //Loads the gazebo controller
  public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
  virtual void Init();

  //Method to be executed every update cycle
  private:virtual void OnUpdate();

  private:
  event::ConnectionPtr updateConnection;
  
  // for setting ROS name space and robot description
  std::string robot_namespace_;
  std::string robot_description_parameter_;

  physics::JointController *jnt_pos_con;

  //map of jointName<String>->Joint positions<double>
  std::map<std::string, double> mtm_joints;

  //vector of joint_names
  std::vector<std::string> mtm_joint_names;

  // parent Model
  physics::ModelPtr myParent;

  // ROS Nodehandle
  ros::NodeHandle* node;

  // ROS Subscriber for JointState messages
  ros::Subscriber sub;
 //callback functing for JointState which are received by sub ROS subscriber
  void joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg);

  // joint state publishes publishing JointState messages
  ros::Publisher joint_state_publisher;
  //function publishing joint stses

 };

}

#endif

