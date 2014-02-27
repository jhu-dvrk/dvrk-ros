// Zihan Chen
// 2013-07-14
// Brief: da Vinci teleop
// 50 Hz


#include <iostream>

#include <ros/ros.h>
#include "dvrk_teleop/mtsTeleop.h"

int main(int argc, char** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // ros initialization
    ros::init(argc, argv, "dvrk_teleop");

    mtsComponentManager * manager = mtsManagerLocal::GetInstance();

    mtsTeleop teleop("dvrk_teleop", 20 * cmn_ms);
    manager->AddComponent(&teleop);

    manager->CreateAllAndWait(2.0 * cmn_s);
    manager->StartAllAndWait(2.0 * cmn_s);

    // ros::spin() callback for subscribers
    std::cerr << "Hit Ctrl-c to quit" << std::endl;
    ros::spin();

    manager->KillAllAndWait(2.0 * cmn_s);
    manager->Cleanup();

    cmnLogger::Kill();

    return 0;
}
