/*
*  Copyright (c) 2013, Nirav Patel, AIM Lab, Worcester Polytechnic Institute
*  All rights reserved.
*
*  This software is provided "as is" under an open source license, with
*  no warranty. 
*
*  daVinciGazeboJointController.cpp
*
*  Created on: Feb 2, 2013
*  Author: Nirav Patel
*/

#include <limits>
#include <math.h>

#include <angles/angles.h>
#include <urdf/model.h>
#include <stdio.h>
#include "daVinciGazeboJointController.h"
#include <boost/bind.hpp>
#include <math/Angle.hh>
#include <physics/JointController.hh>
#include <common/Time.hh>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "daVinciGazeboPlugins/SetJointState.h"

namespace gazebo
{

	GZ_REGISTER_MODEL_PLUGIN(daVinciGazeboJointController);

	//Constructor
	daVinciGazeboJointController::daVinciGazeboJointController()
	{
		// Start up ROS
		std::string name = "daVinciGazeboJointControllerNode";
		int argc = 0;
		ros::init(argc, NULL, name);
		this->jc = NULL;
	}

	// Destructor
	daVinciGazeboJointController::~daVinciGazeboJointController()
	{
		delete this->node;
	}

	// Load the controller
	void daVinciGazeboJointController::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
		printf("Loading daVinciJointPositionCntroller Plugin\n");
		this->myParent = _parent;

		if (!this->myParent)
		{
			gzthrow("daVinciGazeboJointController requires a Model as its parent");
		}
		this->robot_namespace_ = "";
		if(_sdf->HasElement("robotNamespace"))
		{
			this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";
			//printf("Robotname space =%s \n" , this->robot_namespace_.c_str());
		}

		this->robot_description_parameter_ = "";
		if (_sdf->HasElement("robot"))
		{
			this->robot_description_parameter_ = _sdf->GetElement("robot")->GetValueString() ;
			//printf("Robot description parameters =%s \n" , this->robot_description_parameter_.c_str());
		}

		// ROS Nodehandle and puvblish the topic
		this->node = new ros::NodeHandle("~");

		// ROS Subscriber which subsribes to JointState messages
		this->sub = this->node->subscribe<sensor_msgs::JointState>("/psm1/joint_state", 1000, &daVinciGazeboJointController::ROSCallback, this );

		//JointState publisher
		this->joint_state_publisher = this->node->advertise<sensor_msgs::JointState>("/daVinci/set_joint_states", 1000);

		//initialize SetJointState service
		std::string set_joint_states_service_name("set_joint_states");
		ros::AdvertiseServiceOptions set_joint_states_aso = ros::AdvertiseServiceOptions::create<daVinciGazeboPlugins::SetJointState>(
		set_joint_states_service_name,boost::bind(&daVinciGazeboJointController::UpdateJointStates,this,_1,_2),
		ros::VoidPtr(), NULL);

		this->SetJointStateService = this->node->advertiseService(set_joint_states_aso);

		this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&daVinciGazeboJointController::OnUpdate , this));

		//reset joint controler to NULL and store joint position information with all joints set to position 0
		this->jc = NULL;

		int num_joints = this->myParent->GetJointCount();
		for(int i=0 ; i<num_joints ; i++)
		{
			physics::JointPtr jnt = this->myParent->GetJoint(i);
			if( jnt!=NULL )
			{
				this->joint_positions[this->myParent->GetJoint(i)->GetName()] = 0 ;
				printf( "Joint %d is %s \n " , i , this->myParent->GetJoint(i)->GetName().c_str());
			}
		}

	}

	bool daVinciGazeboJointController::UpdateJointStates(daVinciGazeboPlugins::SetJointState::Request  &req,daVinciGazeboPlugins::SetJointState::Response &res)
	{
		printf("UpdateJointStates service called...\n");

		//put these new joint values in the joint_positions map which is refered by joint_controller
		for( unsigned int i=0;i<req.joint_state.name.size();i++)
		{
			this->joint_positions[req.joint_state.name[i]] = req.joint_state.position[i];
			if( req.joint_state.name[i] == "left_arm_outer_pitch_base_joint") //this is parallel link mechanism so we have to simulate it
			{
				this->joint_positions["left_arm_outer_pitch_front_joint"] = req.joint_state.position[i];
				this->joint_positions["left_arm_outer_pitch_bottom_joint"] = -req.joint_state.position[i];
				this->joint_positions["left_arm_outer_pitch_top_joint"] = -req.joint_state.position[i];
				this->joint_positions["left_arm_outer_insertion_joint"] = req.joint_state.position[i];
			}
			if( req.joint_state.name[i] == "right_arm_outer_pitch_base_joint") //this is parallel link mechanism so we have to simulate it
			{
				this->joint_positions["right_arm_outer_pitch_front_joint"] = req.joint_state.position[i];
				this->joint_positions["right_arm_outer_pitch_bottom_joint"] = -req.joint_state.position[i];
				this->joint_positions["right_arm_outer_pitch_top_joint"] = -req.joint_state.position[i];
				this->joint_positions["right_arm_outer_insertion_joint"] = req.joint_state.position[i];
			}
		}	
		res.success = true;
		res.status_message = "Success";
		return true;
	}

	void daVinciGazeboJointController::ROSCallback(const sensor_msgs::JointState::ConstPtr& msg)
	{
		printf("subscriber got a new messge\n");

		//put these new joint values in the joint_positions map which is refered by joint_controller
		for( unsigned int i=0;i<msg->name.size();i++)
		{
			this->joint_positions[msg->name[i]] = msg->position[i];
			if( msg->name[i] == "left_arm_outer_pitch_base_joint") //this is parallel link mechanism so we have to simulate it
			{
				this->joint_positions["left_arm_outer_pitch_front_joint"] = msg->position[i];
				this->joint_positions["left_arm_outer_pitch_bottom_joint"] = -msg->position[i];
				this->joint_positions["left_arm_outer_pitch_top_joint"] = -msg->position[i];
				this->joint_positions["left_arm_outer_insertion_joint"] = msg->position[i];
			}
			if( msg->name[i] == "right_arm_outer_pitch_base_joint") //this is parallel link mechanism so we have to simulate it
			{
				this->joint_positions["right_arm_outer_pitch_front_joint"] = msg->position[i];
				this->joint_positions["right_arm_outer_pitch_bottom_joint"] = -msg->position[i];
				this->joint_positions["right_arm_outer_pitch_top_joint"] = -msg->position[i];
				this->joint_positions["right_arm_outer_insertion_joint"] = msg->position[i];
			}
		}	
	}

	// Initialize the controller
	void daVinciGazeboJointController::Init()
	{
	//nothing to do here
	}

	// Update the controller
	void daVinciGazeboJointController::OnUpdate()
	{
		ros::spinOnce();

		//putting everything here in this function as things are not working when they are 
		//put in different functions

		//this does not work on Load , dont know why so putting it here	
		//	if( this->jc == NULL )
		{
			physics::JointController j_c= physics::JointController(this->myParent);
			this->jc = &j_c;

			int num_joints = this->myParent->GetJointCount();
			//	printf("Number of joints=%d\n" , num_joints);

			for(int i=0 ; i<num_joints ; i++)
			{
				physics::JointPtr jnt = this->myParent->GetJoint(i);
				if( jnt!=NULL )
				{
					this->jc->AddJoint(jnt);
					this->jc->SetJointPosition( jnt->GetName() , this->joint_positions[jnt->GetName()] );

					//printf( "Joint %d is %s \n " , i , this->myParent->GetJoint(i)->GetName().c_str());
				}
			}
		}

		//publish Joint State message

		//PublishJointStates(); //Not publishing for time being
	}

	void daVinciGazeboJointController::PublishJointStates()
	{
		sensor_msgs::JointState joint_states_msg;

		//define message values
		joint_states_msg.name.resize(6);
		joint_states_msg.position.resize(6);
		joint_states_msg.velocity.resize(6);
		joint_states_msg.effort.resize(6);

		joint_states_msg.name[0] = "left_arm_outer_yaw_joint";
		joint_states_msg.name[1] = "left_arm_outer_pitch_base_joint";
		joint_states_msg.name[2] = "left_arm_tool_insertion_joint";

		joint_states_msg.name[3] = "right_arm_outer_yaw_joint";
		joint_states_msg.name[4] = "right_arm_outer_pitch_base_joint";
		joint_states_msg.name[5] = "right_arm_tool_insertion_joint";

		physics::JointPtr  left_outer_yaw_joint = this->myParent->GetJoint(joint_states_msg.name[0]);
		physics::JointPtr  left_outer_pitch_base_joint = this->myParent->GetJoint(joint_states_msg.name[1]);
		physics::JointPtr  left_tool_insertion_joint = this->myParent->GetJoint(joint_states_msg.name[2]);


		physics::JointPtr  right_outer_yaw_joint = this->myParent->GetJoint(joint_states_msg.name[3]);
		physics::JointPtr  right_outer_pitch_base_joint = this->myParent->GetJoint(joint_states_msg.name[4]);
		physics::JointPtr  right_tool_insertion_joint = this->myParent->GetJoint(joint_states_msg.name[5]);

		joint_states_msg.position[0] = (int)(left_outer_yaw_joint->GetAngle(0).GetAsDegree());
		joint_states_msg.position[1] = (int)(left_outer_pitch_base_joint->GetAngle(0).GetAsDegree());
		joint_states_msg.position[2] = (int)(left_tool_insertion_joint->GetAngle(0).GetAsDegree());

		joint_states_msg.position[3] = (int)(right_outer_yaw_joint->GetAngle(0).GetAsDegree());
		joint_states_msg.position[4] = (int)(right_outer_pitch_base_joint->GetAngle(0).GetAsDegree());
		joint_states_msg.position[5] = (int)(right_tool_insertion_joint->GetAngle(0).GetAsDegree());

		joint_states_msg.velocity[0] = left_outer_yaw_joint->GetVelocity(0);
		joint_states_msg.velocity[1] = left_outer_pitch_base_joint->GetVelocity(0);
		joint_states_msg.velocity[2] = left_tool_insertion_joint->GetVelocity(0);


		joint_states_msg.velocity[3] = right_outer_yaw_joint->GetVelocity(0);
		joint_states_msg.velocity[4] = right_outer_pitch_base_joint->GetVelocity(0);
		joint_states_msg.velocity[5] = right_tool_insertion_joint->GetVelocity(0);

		joint_states_msg.effort[0] = left_outer_yaw_joint->GetForce(0);
		joint_states_msg.effort[1] = left_outer_pitch_base_joint->GetForce(0);
		joint_states_msg.effort[2] = left_tool_insertion_joint->GetForce(0);


		joint_states_msg.effort[3] = right_outer_yaw_joint->GetForce(0);
		joint_states_msg.effort[4] = right_outer_pitch_base_joint->GetForce(0);
		joint_states_msg.effort[5] = right_tool_insertion_joint->GetForce(0);

		//ROS_INFO("Publishing JointState message......\n");

		this->joint_state_publisher.publish(joint_states_msg);

		ros::spinOnce();
	}
	//tried using this but does not work as we can not declare instance of JointController class
	void daVinciGazeboJointController::InitializeJointControllers()
	{
		physics::JointController j_c= physics::JointController(this->myParent);
		this->jc = &j_c;

		int num_joints = this->myParent->GetJointCount();
		//	printf("Number of joints=%d\n" , num_joints);

		for(int i=0 ; i<num_joints ; i++)
		{
			physics::JointPtr jnt = this->myParent->GetJoint(i);
			if( jnt!=NULL )
			{
				this->jc->AddJoint(jnt);
				//this->jc->SetJointPosition( jnt->GetName() , 0 );

				printf( "Joint %d is %s \n " , i , this->myParent->GetJoint(i)->GetName().c_str());
			}
		}
	}

}
