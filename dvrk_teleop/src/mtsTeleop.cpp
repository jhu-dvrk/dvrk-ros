#include <iostream>
#include <cisstRobot/robManipulator.h>
#include <cisstVector.h>

#include <cisst_ros_bridge/mtsCISSTToROS.h>
#include <cisst_ros_bridge/mtsROSToCISST.h>

#include "dvrk_teleop/mtsTeleop.h"


CMN_IMPLEMENT_SERVICES_DERIVED(mtsTeleop, mtsTaskPeriodic);


mtsTeleop::mtsTeleop(const std::string &name, const double &period):
    mtsTaskPeriodic(name, period, 256),
    has_clutch_(false),
    is_clutch_pressed_(false),
    is_enabled_(false),
    counter_(0),
    counter_master_cb_(0),
    counter_slave_cb_(0)
{
    // subscriber
    // NOTE: queue size is set to 1 to make sure data is fresh
    sub_teleop_enable_ = nh_.subscribe("/dvrk_teleop/enable", 1,
                                  &mtsTeleop::teleop_enable_cb, this);
    sub_mtm_pose_ = nh_.subscribe("/dvrk_mtm/cartesian_pose_current", 1,
                                  &mtsTeleop::master_pose_cb, this);
    sub_psm_pose_ = nh_.subscribe("/dvrk_psm/cartesian_pose_current", 1,
                                  &mtsTeleop::slave_pose_cb, this);
    sub_foodpedal_clutch_ = nh_.subscribe("/dvrk_footpedal/clutch_state", 1,
                                          &mtsTeleop::footpedal_clutch_cb, this);


    // publisher
    pub_mtm_pose_ = nh_.advertise<geometry_msgs::Pose>("/dvrk_mtm/cartesian_pose_command", 1);
    pub_psm_pose_ = nh_.advertise<geometry_msgs::Pose>("/dvrk_psm/cartesian_pose_command", 1);
}

void mtsTeleop::Configure(const std::string &)
{
}

void mtsTeleop::Startup(void) {
}

void mtsTeleop::Run(void)
{
    // increment counter
    counter_++;

    // cisst process queued commands
    ProcessQueuedCommands();

    // refresh data
    ros::spinOnce();

    // publish
    vctMatRot3 mtm2psm;
    mtm2psm.Assign(-1.0, 0.0, 0.0,
                   0.0,-1.0, 0.0,
                   0.0, 0.0, 1.0);

    vctMatRot3 psm6tomtm7;
    psm6tomtm7.Assign(0.0, 0.0, 1.0,
                      1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0);

    if (is_enabled_) {

        // assign current psm pose
        psm_pose_cmd_.Assign(psm_pose_cur_);

        // translation
        double scale = 0.2;
        vct3 mtm_tra = mtm_pose_cur_.Translation() - mtm_pose_pre_.Translation();
        vct3 psm_tra = scale * mtm_tra;

        psm_pose_cmd_.Translation() = psm_pose_cmd_.Translation() + mtm2psm * psm_tra;

        // rotation
        vctMatRot3 psm_motion_rot;
        psm_motion_rot = mtm2psm * mtm_pose_cur_.Rotation() * psm6tomtm7;
        psm_pose_cmd_.Rotation().FromNormalized(psm_motion_rot);

    } else {
        // In this mode, MTM orientation follows PSM orientation

        // keep current pose
        psm_pose_cmd_.Assign(psm_pose_cur_);

        // mtm needs to follow psm orientation
        mtm_pose_cmd_.Assign(mtm_pose_cur_);
        vctMatRot3 mtm_rot_cmd;
        mtm_rot_cmd = mtm2psm.Inverse() * psm_pose_cur_.Rotation() * psm6tomtm7.Inverse();
        mtm_pose_cmd_.Rotation().FromNormalized(mtm_rot_cmd);
    }

    mtsCISSTToROS(psm_pose_cmd_, msg_psm_pose_);
    pub_psm_pose_.publish(msg_psm_pose_);

    mtsCISSTToROS(mtm_pose_cmd_, msg_mtm_pose_);
    pub_mtm_pose_.publish(msg_mtm_pose_);

    // save current pose as previous
    mtm_pose_pre_.Assign(mtm_pose_cur_);
}

void mtsTeleop::Cleanup(void) {
}

void mtsTeleop::teleop_enable_cb(const std_msgs::BoolConstPtr &msg)
{
    is_enabled_ = msg->data;
}

void mtsTeleop::master_pose_cb(const geometry_msgs::PoseConstPtr &msg)
{
    counter_master_cb_++;
    mtsROSToCISST((*msg), mtm_pose_cur_);

    // initialize mtm_pose_pre_
    if (counter_master_cb_ == 1) {
        mtm_pose_pre_.Assign(mtm_pose_cur_);
    }
}

void mtsTeleop::slave_pose_cb(const geometry_msgs::PoseConstPtr &msg)
{
    counter_slave_cb_++;
    mtsROSToCISST((*msg), psm_pose_cur_);
}

void mtsTeleop::footpedal_clutch_cb(const std_msgs::BoolConstPtr &msg)
{
    has_clutch_ = true;
    // here
    if (msg->data == true) {
        is_clutch_pressed_ = false;
    } else {
        is_clutch_pressed_ = true;
    }
}
