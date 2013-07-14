// Zihan Chen
// 2013-07-14
// Brief: da Vinci psm kinematics


#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <cisstRobot/robManipulator.h>
#include <cisstVector.h>

// set up joint state variables
vctDoubleVec psm_joint_current;
vctDoubleVec psm_joint_command;
vctFrm4x4 psm_pose_current;
vctFrm4x4 psm_pose_command;


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

//    ROS_ERROR(psm_joint_current.ToString().c_str());
}

void cisstToROS(const vctFrm4x4 &cisstData, geometry_msgs::Pose &rosData)
{
    vctQuatRot3 quat(cisstData.Rotation(), VCT_NORMALIZE);
    rosData.orientation.x = quat.X();
    rosData.orientation.y = quat.Y();
    rosData.orientation.z = quat.Z();
    rosData.orientation.w = quat.W();
    rosData.position.x = cisstData.Translation().X();
    rosData.position.y = cisstData.Translation().Y();
    rosData.position.z = cisstData.Translation().Z();
}

int main(int argc, char** argv)
{
    // ros initialization
    ros::init(argc, argv, "irk_psm_kinematics");
    ros::NodeHandle nh;
    ros::Rate rate(50);  // 50 hz rate

    // subscriber
    ros::Subscriber sub_psm_cmd =
            nh.subscribe("/irk_psm/cartesian_pose_command", 1000, psm_cmd_pose_cb);

    ros::Subscriber sub_psm_fb =
            nh.subscribe("/joint_states", 1000, psm_joint_feedback_cb);

    // publisher
    ros::Publisher pub_psm_joint_state =
            nh.advertise<sensor_msgs::JointState>("/irk_psm/joint_states", 1000);
    ros::Publisher pub_psm_pose =
            nh.advertise<geometry_msgs::Pose>("/irk_psm/cartesian_pose", 1000);


    // cisst robManipulator
    std::string filename = ros::package::getPath("irk_psm_kinematics");
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
//    msg.name.push_back("outer_wrist_open_angle_joint_1");

    geometry_msgs::Pose msg_pose;

    int count = 0;
    double deltaX = 0.001;

    // ------------ run() --------------------------
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {

        // --------- Compute current pose & publish ----------
        // psm forward kinematics
        psm_pose_current = psm_manip.ForwardKinematics(psm_joint_current);
        cisstToROS(psm_pose_current, msg_pose);
        if (count % 10 == 0) {
//            std::cout << psm_pose_current << std::endl << std::endl;
        }

        // publish current pose
        pub_psm_pose.publish(msg_pose);

        // ---------- Compute command psm joint positin -----------

        // 2 sec, set robot to start position
        // after, in teleop mode
        if ((ros::Time::now() - start_time).sec < 2.0) {
            psm_joint_command.SetAll(0.0);
            psm_joint_command[2] = 0.10;
            psm_pose_command.Assign(psm_pose_current);
        } else {
//            vctFrm4x4 psm_pose_command(psm_pose_current);
//            psm_pose_command.Translation().X() += deltaX;
//            if (psm_pose_command.Translation().X() > 0.08) {
//                deltaX = -0.001;
//            } else if (psm_pose_command.Translation().X() < -0.08) {
//                deltaX = 0.001;
//            }
//            psm_joint_command.ForceAssign(psm_joint_current);
            // psm_pose_command updated in callback
            psm_manip.InverseKinematics(psm_joint_command, psm_pose_command);
        }

        msg_js.position.resize(psm_joint_command.size());
        std::copy(psm_joint_command.begin(), psm_joint_command.end(), msg_js.position.begin());

        // publish command psm_joint_state
        pub_psm_joint_state.publish(msg_js);

        ros::spinOnce();

        rate.sleep();
        count++;
    }

    return 0;
}
