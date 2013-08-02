// Zihan Chen
// 2013-07-14
// Brief: da Vinci teleop


#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

#include "irk_teleop/irkTeleopQWidget.h"


int main(int argc, char** argv)
{
    // ros initialization
    ros::init(argc, argv, "irk_teleop_gui");
    QApplication qapp(argc, argv);

    irkTeleopQWidget gui("irk_teleop_gui", 20 * cmn_ms);

    gui.show();   // show gui widget
    qapp.exec();  //

    cmnLogger::Kill();

    return 0;
}