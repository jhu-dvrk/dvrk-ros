#ifndef _mtsTeleop_h
#define _mtsTeleop_h

#include <cisstMultiTask/mtsTaskPeriodic.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

// set up joint state variables
class mtsTeleop: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleop(const std::string &name, const double &period);
    ~mtsTeleop(){}

    void Configure(const std::string &);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:

    void teleop_enable_cb(const std_msgs::BoolConstPtr &msg);
    void master_pose_cb(const geometry_msgs::PoseConstPtr &msg);
    void slave_pose_cb(const geometry_msgs::PoseConstPtr &msg);
    void footpedal_clutch_cb(const std_msgs::BoolConstPtr &msg);

    bool has_clutch_;
    bool is_clutch_pressed_;
    bool is_enabled_;

    size_t counter_;
    size_t counter_master_cb_;
    size_t counter_slave_cb_;

    vctFrm4x4 mtm_pose_cur_;
    vctFrm4x4 psm_pose_cur_;
    vctFrm4x4 mtm_pose_pre_;

    vctFrm4x4 psm_pose_cmd_;
    vctFrm4x4 mtm_pose_cmd_;
    geometry_msgs::Pose msg_psm_pose_;
    geometry_msgs::Pose msg_mtm_pose_;

    // ros variables
    ros::NodeHandle nh_;

    // subscribers
    ros::Subscriber sub_teleop_enable_;
    ros::Subscriber sub_mtm_pose_;
    ros::Subscriber sub_psm_pose_;
    ros::Subscriber sub_foodpedal_clutch_;

    ros::Publisher pub_psm_pose_;
    ros::Publisher pub_mtm_pose_;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleop);

#endif // _mtsTeleop_h
