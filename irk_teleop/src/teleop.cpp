// Zihan Chen
// 2013-07-14
// Brief: da Vinci teleop


#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

#include <cisstRobot/robManipulator.h>
#include <cisstVector.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>

#include <sawROS/mtsROSToCISST.h>
#include <sawROS/mtsCISSTToROS.h>

// set up joint state variables

class mtsTeleop: public mtsTaskPeriodic
{
public:
    mtsTeleop(const std::string &name, const double &period):
        mtsTaskPeriodic(name, period, 256),
        is_clutch_pressed_(false),
        counter_(0)
    {
        delta = 0.0005;

        // subscriber
        // NOTE: queue size is set to 1 to make sure data is fresh
        sub_mtm_pose_ = nh_.subscribe("/irk_mtm/cartesian_pose_current", 1,
                                        &mtsTeleop::master_pose_cb, this);
        sub_psm_pose_ = nh_.subscribe("/irk_psm/cartesian_pose_current", 1,
                                       &mtsTeleop::slave_pose_cb, this);
        sub_foodpedal_clutch_ = nh_.subscribe("/irk_footpedal/clutch_state", 1,
                                               &mtsTeleop::footpedal_clutch_cb, this);

        // publisher
        pub_pose_ = nh_.advertise<geometry_msgs::Pose>("/irk_psm/cartesian_pose_command", 1);
    }

    ~mtsTeleop(){}

    void Configure(const std::string &)
    {
    }

    void Startup(void) {
    }

    void Run(void) {
        // cisst process queued commands
        ProcessQueuedCommands();

        // refresh data
        ros::spinOnce();

        // publish
#if 1
        if (counter_%10 == 0) {
//            std::cerr << "clutch = " << is_clutch_pressed_ << std::endl;
            std::cerr << " mtm = " << std::endl
                      << mtm_pose_cur_ << std::endl;
        }
#endif

        if (is_clutch_pressed_) {
            std::cerr << "pressed" << std::endl;

            // translation
            double scale = 1.0;
            vct3 mtm_tra = mtm_pose_cur_.Translation() - mtm_pose_pre_.Translation();
            vct3 psm_tra = mtm_tra * scale;
            vctMatRot3 mtm2psm;
            mtm2psm.Assign(-1.0, 0.0, 0.0,
                            0.0,-1.0, 0.0,
                            0.0, 0.0, 1.0);
            psm_tra = psm_tra * mtm2psm;
            psm_pose_cmd_.Translation() = psm_pose_cur_.Translation() + psm_tra;

            // rotation
            vctFrm4x4 mtm_motion;
            mtm_motion = mtm_pose_pre_.Inverse() * mtm_pose_cur_;
            vctAxAnRot3 mtm_motion_rot;
            mtm_motion_rot.FromNormalized(mtm_motion.Rotation());
            vctFrm4x4 mtm_wrt_psm;
            mtm_wrt_psm = psm_pose_pre_.Inverse() * mtm_pose_pre_;

            vct3 psm_rot_axis;
            psm_rot_axis = mtm_wrt_psm.Rotation() * mtm_motion_rot.Axis();
            vctMatRot3 psm_motion_rot(vctAxAnRot3(psm_rot_axis, mtm_motion_rot.Angle()));

            psm_motion_rot = psm_pose_pre_.Rotation() * psm_motion_rot;
            psm_motion_rot = mtm2psm * psm_motion_rot;

            psm_pose_cmd_.Rotation().FromNormalized(psm_motion_rot);

        } else {
            psm_pose_cmd_.Assign(psm_pose_cur_);

#if 0
            // NOTE: testing teleop
            psm_pose_cmd_.Translation().X() += delta;
            if (psm_pose_cur_.Translation().X() > 0.07) {
                delta = -0.0005;
            } else if (psm_pose_cur_.Translation().X() < -0.07) {
                delta = 0.0005;
            }
#endif
        }

        mtsCISSTToROS(psm_pose_cmd_, msg_psm_pose_);
        pub_pose_.publish(msg_psm_pose_);

        // save previous
        mtm_pose_pre_.Assign(mtm_pose_cur_);
        psm_pose_pre_.Assign(psm_pose_pre_);

#if 0
        if (counter_%10 == 0) {
            std::cerr << " mtm pre = " << std::endl
                      << mtm_pose_pre_ << std::endl;
        }
#endif

        counter_++;
    }

    void Cleanup(void) {
    }


protected:

    void master_pose_cb(const geometry_msgs::PoseConstPtr &msg)
    {
        mtsROSToCISST((*msg), mtm_pose_cur_);
    }

    void slave_pose_cb(const geometry_msgs::PoseConstPtr &msg)
    {
        mtsROSToCISST((*msg), psm_pose_cur_);
    }

    void footpedal_clutch_cb(const std_msgs::BoolConstPtr &msg)
    {
        // here
        if (msg->data == true) {
            is_clutch_pressed_ = false;
        } else {
            is_clutch_pressed_ = true;
        }
    }

    bool is_clutch_pressed_;
    size_t counter_;
    double delta;

    vctFrm4x4 mtm_pose_cur_;
    vctFrm4x4 psm_pose_cur_;
    vctFrm4x4 mtm_pose_pre_;
    vctFrm4x4 psm_pose_pre_;

    vctFrm4x4 psm_pose_cmd_;
    geometry_msgs::Pose msg_psm_pose_;

    // ros variables
    ros::NodeHandle nh_;

    ros::Subscriber sub_mtm_pose_;
    ros::Subscriber sub_psm_pose_;
    ros::Subscriber sub_foodpedal_clutch_;

    ros::Publisher pub_pose_;
};



int main(int argc, char** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // ros initialization
    ros::init(argc, argv, "irk_teleop");

    mtsComponentManager * manager = mtsManagerLocal::GetInstance();

    mtsTeleop teleop("irk_teleop", 20 * cmn_ms);
    manager->AddComponent(&teleop);

    manager->CreateAllAndWait(2.0 * cmn_s);
    manager->StartAllAndWait(2.0 * cmn_s);

    // ros::spin() callback for subscribers
    std::cout << "Hit Ctrl-c to quit" << std::endl;
    ros::spin();

    manager->KillAllAndWait(2.0 * cmn_s);
    manager->Cleanup();

    cmnLogger::Kill();

    return 0;
}
