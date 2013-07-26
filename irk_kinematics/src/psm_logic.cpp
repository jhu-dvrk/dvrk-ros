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


void psm_cmd_pose_cb(const geometry_msgs::Pose &msg)
{
    std::cout << msg.position.x << std::endl;
    psm_pose_command.Translation().X() = msg.position.x;
    psm_pose_command.Translation().Y() = msg.position.y;
    psm_pose_command.Translation().Z() = msg.position.z;

    vctQuatRot3 quat;
    quat.X() = msg.orientation.x;
    quat.Y() = msg.orientation.y;
    quat.Z() = msg.orientation.z;
    quat.W() = msg.orientation.w;

    vctMatRot3 rotation(quat, VCT_NORMALIZE);
    psm_pose_command.Rotation().Assign(rotation);

    std::cout << psm_pose_command << std::endl;
}

void mtm_gripper_cb(const std_msgs::Float32 &msg)
{
    mtm_gripper = msg.data;
}

void psm_joint_feedback_cb(const sensor_msgs::JointState &msg)
{
//    ROS_ERROR("joint_feedback_cb %f", msg.position.at(0));
    psm_joint_current[0] = msg.position[0];  // outer_yaw_joint
    psm_joint_current[1] = msg.position[1];  // outer_pitch_joint_1
    psm_joint_current[2] = msg.position[6];  // outer_insertion_joint
    psm_joint_current[3] = msg.position[7];  // outer_roll_joint
    psm_joint_current[4] = msg.position[8];  // outer_wrist_pitch_joint
    psm_joint_current[5] = msg.position[9];  // outer_wrist_yaw_joint
    //    psm_joint_current[6] = msg.position[10]; // outer_wrist_open_angle_joint_1
}

void psm_mode_cb(const std_msgs::Int8 &msg)
{
    std::cerr << "-------------- GOT ----------" << std::endl;
    if (msg.data >= 0 && msg.data <=2) {
        control_mode = msg.data;
        ROS_WARN("PSM switched to %d", msg.data);
    }
}

int main(int argc, char** argv)
{
    // ros initialization
    ros::init(argc, argv, "irk_psm_kinematics");
    ros::NodeHandle nh;
    ros::Rate rate(100);  // 100 hz rate

    // subscriber
    ros::Subscriber sub_psm_cmd =
            nh.subscribe("/irk_psm/cartesian_pose_command", 1000, psm_cmd_pose_cb);
    ros::Subscriber sub_mtm_gripper =
        nh.subscribe("/irk_mtm/gripper_position", 1000, mtm_gripper_cb);

    ros::Subscriber sub_psm_fb =
            nh.subscribe("/irk_psm/joint_states", 1000, psm_joint_feedback_cb);

    ros::Subscriber sub_mode =
            nh.subscribe("/irk_psm/control_mode", 1000, psm_mode_cb);

    // publisher
    ros::Publisher pub_psm_joint_state =
            nh.advertise<sensor_msgs::JointState>("/irk_psm/joint_states_command", 1000);
    ros::Publisher pub_psm_pose =
            nh.advertise<geometry_msgs::Pose>("/irk_psm/cartesian_pose_current", 1000);


    // cisst robManipulator
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
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {

        // --------- Compute current pose & publish ----------
        // psm forward kinematics
        psm_pose_current = psm_manip.ForwardKinematics(psm_joint_current);
        psm_pose_current = psm_pose_current * frame6to7;
        psm_pose_current.Rotation().NormalizedSelf();

        mtsCISSTToROS(psm_pose_current, msg_pose);
        if (count % 10 == 0) {
//            std::stringstream ss;
//            std::cout << psm_pose_current << std::endl << std::endl;
        }

        // publish current pose
        pub_psm_pose.publish(msg_pose);

        // ---------- Compute command psm joint positin -----------

        // 2 sec, set robot to start position
        // after, in teleop mode
        vctFrm4x4 pose6;
        switch(control_mode)
        {
        case MODE_RESET:
            psm_pose_command.Assign(psm_pose_current);
            psm_joint_command.SetAll(0.0);
            psm_joint_command[2] = 0.10;
            break;
        case MODE_MANUAL:
            psm_pose_command.Assign(psm_pose_current);
            pose6 = psm_pose_command * frame6to7.Inverse();
            psm_manip.InverseKinematics(psm_joint_command, pose6);
            std::cerr << "manual " << psm_joint_command << std::endl;
            break;
        case MODE_TELEOP:
            // psm_pose_command updated in callback!
            psm_pose_current.Assign(psm_pose_current);
            pose6 = psm_pose_command * frame6to7.Inverse();
            psm_manip.InverseKinematics(psm_joint_command, pose6);
            break;
        default:
            break;
        }

        // 6 + open angle (gripper)
        msg_js.position.resize(psm_joint_command.size() + 1);
        std::copy(psm_joint_command.begin(), psm_joint_command.end(), msg_js.position.begin());
        msg_js.position[6] = mtm_gripper;

        if (control_mode != MODE_MANUAL) {
            // publish command psm_joint_state
            pub_psm_joint_state.publish(msg_js);
        }

        ros::spinOnce();

        rate.sleep();
        count++;
    }

    return 0;
}
