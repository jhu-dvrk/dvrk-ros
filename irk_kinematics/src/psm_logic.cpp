// Zihan Chen
// 2013-07-14
// Brief: da Vinci psm kinematics


#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include <cisstRobot/robManipulator.h>
#include <cisstVector.h>

#include <sawROS/mtsCISSTToROS.h>
#include <sawROS/mtsROSToCISST.h>

// set up joint state variables
vctDoubleVec psm_joint_current;
vctDoubleVec psm_joint_command;
vctFrm4x4 psm_pose_current;
vctFrm4x4 psm_pose_command;
double mtm_gripper;
int control_mode;

const int MODE_RESET = 0;
const int MODE_MANUAL = 1;
const int MODE_TELEOP = 2;


// command psm pose from teleop
void psm_cmd_pose_cb(const geometry_msgs::Pose &msg)
{
    mtsROSToCISST(msg, psm_pose_command);
//    std::cout << psm_pose_command << std::endl;
}

// mtm gripper open angle
void mtm_gripper_cb(const std_msgs::Float32 &msg)
{
    mtm_gripper = msg.data;
}

// psm joint feedback from RVIZ (joint_state_publisher)
void psm_joint_feedback_cb(const sensor_msgs::JointState &msg)
{
    if (control_mode != 2) {
        psm_joint_current[0] = msg.position[0];  // outer_yaw_joint
        psm_joint_current[1] = msg.position[1];  // outer_pitch_joint_1
        psm_joint_current[2] = msg.position[6];  // outer_insertion_joint
        psm_joint_current[3] = msg.position[7];  // outer_roll_joint
        psm_joint_current[4] = msg.position[8];  // outer_wrist_pitch_joint
        psm_joint_current[5] = msg.position[9];  // outer_wrist_yaw_joint
        //    psm_joint_current[6] = msg.position[10]; // outer_wrist_open_angle_joint_1
    }
}

void psm_mode_cb(const std_msgs::Int8 &msg)
{
    std::cerr << "----- PSM GOT ------" << std::endl;
    if (msg.data >= 0 && msg.data <=2) {
        control_mode = msg.data;
        ROS_WARN("PSM switched to %d", msg.data);
    }

    // mode = TELEOP
    if (msg.data == 2) {
        psm_joint_command.Assign(psm_joint_current);
    }
}

int main(int argc, char** argv)
{
    // ros initialization
    ros::init(argc, argv, "irk_psm_kinematics");
    ros::NodeHandle nh;
    ros::Rate rate(200);  // 100 hz rate

    // subscriber
    ros::Subscriber sub_psm_cmd =
            nh.subscribe("/irk_psm/cartesian_pose_command", 1, psm_cmd_pose_cb);
    ros::Subscriber sub_mtm_gripper =
        nh.subscribe("/irk_mtm/gripper_position", 1, mtm_gripper_cb);

    ros::Subscriber sub_psm_fb =
            nh.subscribe("/irk_psm/joint_states", 1, psm_joint_feedback_cb);

    ros::Subscriber sub_mode =
            nh.subscribe("/irk/control_mode", 1, psm_mode_cb);

    // publisher
    ros::Publisher pub_psm_joint_state_cmd =
            nh.advertise<sensor_msgs::JointState>("/irk_psm/joint_states_command", 1);
    ros::Publisher pub_psm_pose_current =
            nh.advertise<geometry_msgs::Pose>("/irk_psm/cartesian_pose_current", 1);


    // --- cisst robManipulator ---
    std::string filename = ros::package::getPath("irk_kinematics");
    filename.append("/config/dvpsm.rob");
    robManipulator psm_manip;
    robManipulator::Errno result;
    result = psm_manip.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        ROS_ERROR("failed to load manipulator config file: %s", filename.c_str());
    } else {
        ROS_INFO("loaded psm manipulator");
    }

    // --- initialize joint current/command -----
    psm_joint_current.SetSize(6);  // 6 + 1 (open angle)
    psm_joint_current.SetAll(0.0);
    psm_joint_command.ForceAssign(psm_joint_current);

    sensor_msgs::JointState msg_js;
    msg_js.name.clear();
    msg_js.name.push_back("outer_yaw_joint");
    msg_js.name.push_back("outer_pitch_joint_1");
    msg_js.name.push_back("outer_insertion_joint");
    msg_js.name.push_back("outer_roll_joint");
    msg_js.name.push_back("outer_wrist_pitch_joint");
    msg_js.name.push_back("outer_wrist_yaw_joint");
    msg_js.name.push_back("outer_wrist_open_angle_joint_1");

    geometry_msgs::Pose msg_pose;

    int count = 0;
    control_mode = MODE_RESET;  // start with reset_mode
    vctFrm4x4 frame6to7;
    frame6to7.Assign(0.0, -1.0, 0.0, 0.0,
                     0.0,  0.0, 1.0, 0.0102,
                     -1.0, 0.0, 0.0, 0.0,
                     0.0,  0.0, 0.0, 1.0);

    // ------------ run() --------------------------
    while (ros::ok()) {

        // --------- Compute current pose & publish ----------
        // psm forward kinematics
        psm_pose_current = psm_manip.ForwardKinematics(psm_joint_current);
        psm_pose_current = psm_pose_current * frame6to7;
        psm_pose_current.Rotation().NormalizedSelf();
        mtsCISSTToROS(psm_pose_current, msg_pose);

        // publish current pose
        pub_psm_pose_current.publish(msg_pose);

        // ---------- Compute command psm joint positin -----------
        vctFrm4x4 pose6;
        switch(control_mode)
        {
        case MODE_RESET:
            psm_joint_command.SetAll(0.0);
            psm_joint_command[2] = 0.10;
            psm_pose_command.Assign(psm_pose_current);
            break;
        case MODE_MANUAL:
            // do nothing for MANUAL
            // controlled using Slifer GUI
            psm_pose_command.Assign(psm_pose_current);
            break;
        case MODE_TELEOP:
            // psm_pose_command updated in callback!
            pose6 = psm_pose_command * frame6to7.Inverse();
            psm_manip.InverseKinematics(psm_joint_command, pose6);
            psm_joint_current.Assign(psm_joint_command);

//            ROS_WARN(psm_joint_command.ToString().c_str());
            break;
        default:
            break;
        }

        // -----  Assign command joint state and Publish --------
        // 6 + open angle (gripper)
        msg_js.position.resize(psm_joint_command.size() + 1);
        std::copy(psm_joint_command.begin(), psm_joint_command.end(), msg_js.position.begin());
        msg_js.position[6] = mtm_gripper;

        if (control_mode != MODE_MANUAL) {
            pub_psm_joint_state_cmd.publish(msg_js);
        }

        ros::spinOnce();
        rate.sleep();
        count++;
    }

    return 0;
}
