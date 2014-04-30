/*
*  Copyright (c) 2014, Adnan Munawar, AIM Lab, Worcester Polytechnic Institute
*  All rights reserved.
*
*  This software is provided "as is" under an open source license, with
*  no warranty. 
*
*  dvrk_gazebo_controller.cpp
*
*  Created on: April 27, 2014
*  Author: Adnan Munawar
*  Old_Support : Nirav Patel
*/

#include "dvrk_gazebo_controller.h"

namespace gazebo
{

    GZ_REGISTER_MODEL_PLUGIN(dvrk_gazebo_controller);

	//Constructor
    dvrk_gazebo_controller::dvrk_gazebo_controller()
	{
		// Start up ROS
        std::string name = "dv_jnt_pos_con_Node";
		int argc = 0;
		ros::init(argc, NULL, name);
        this->jnt_pos_con = NULL;

        this->mtm_joint_names.resize(8);
        this->mtm_joint_names[0] = "right_outer_yaw_joint";
        this->mtm_joint_names[1] = "right_shoulder_pitch_joint";
        this->mtm_joint_names[2] = "right_elbow_pitch_joint";
        this->mtm_joint_names[3] = "right_wrist_platform_joint";
        this->mtm_joint_names[4] = "right_wrist_pitch_joint";
        this->mtm_joint_names[5] = "right_wrist_yaw_joint";
        this->mtm_joint_names[6] = "right_wrist_roll_joint";
        this->mtm_joint_names[7] = "right_shoulder_pitch_parallel_joint";

        for (int i = 0 ; i < this->mtm_joint_names.size() ; i++)
        {
            this->mtm_joints[this->mtm_joint_names.at(i)] = 0 ;
        }
    }

	// Destructor
    dvrk_gazebo_controller::~dvrk_gazebo_controller()
	{
		delete this->node;
	}

	// Load the controller
    void dvrk_gazebo_controller::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
        printf("Loading dv_adnan_controller \n");
		this->myParent = _parent;
        this->jnt_pos_con = new physics::JointController(this->myParent);

		if (!this->myParent)
		{
            gzthrow("dvrk_gazebo_controller requires a Model as its parent");
		}

		// ROS Nodehandle and puvblish the topic
		this->node = new ros::NodeHandle("~");

		// ROS Subscriber which subsribes to JointState messages
        this->sub = this->node->subscribe<sensor_msgs::JointState>(
                    "/gazebo_mtm/joint_position_current", 1000, &dvrk_gazebo_controller::joint_state_cb, this );

		//JointState publisher
        this->joint_state_publisher = this->node->advertise<sensor_msgs::JointState>("/daVinci/set_joint_states", 1000);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&dvrk_gazebo_controller::OnUpdate , this));

		//reset joint controler to NULL and store joint position information with all joints set to position 0

        int num_joints = this->myParent->GetJointCount();
        if(num_joints != this->mtm_joint_names.size())
        {
            ROS_WARN("Number of joints in gazebo: %d don't match the pre-specified number of joints: %lu",
                      num_joints, this->mtm_joint_names.size());
        }
        for(int i=0 ; i<this->mtm_joint_names.size() ; i++)
		{
            physics::JointPtr jnt = this->myParent->GetJoint(this->mtm_joint_names.at(i));
			if( jnt!=NULL )
			{
                ROS_INFO("Loaded controller for %s",jnt->GetName().c_str());
                this->mtm_joints[this->mtm_joint_names.at(i)] = 0 ;
			}
        }


	}


    void dvrk_gazebo_controller::joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg)
	{
        {
            for(int i=0 ; i < msg->position.size() ; i++)
            {
                this->mtm_joints.at(msg->name.at(i)) = msg->position.at(i);
            }
        }
	}

	// Initialize the controller
    void dvrk_gazebo_controller::Init()
	{
	//nothing to do here
	}

	// Update the controller
    void dvrk_gazebo_controller::OnUpdate()
	{
		ros::spinOnce();
        {

            for(int i=0 ; i<this->mtm_joint_names.size() ; i++)
			{
                physics::JointPtr jnt = this->myParent->GetJoint(this->mtm_joint_names.at(i));
                if( jnt!=NULL )
                {
                    this->jnt_pos_con->SetJointPosition(jnt,
                                                        this->mtm_joints.at(this->mtm_joint_names.at(i)));
                }
			}
		}
    }


}
