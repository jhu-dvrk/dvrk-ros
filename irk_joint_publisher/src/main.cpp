/* License here */

// system
#include <iostream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// cisst/saw
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnPath.h>
#include <cisstOSAbstraction/osaSleep.h>

#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstParameterTypes/prmPositionJointGet.h>

#include <sawROS/mtsROSBridge.h>


int main(int argc, char ** argv)
{
    // gcmip & processname
    std::string gcmip;
    std::string processname = "dv_joint_control";  // ros node name

    // ros initialization
    ros::init(argc, argv, processname);

    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ALL);

    // create task manager
    mtsManagerLocal* LCM = NULL;
    if (argc == 2) {
        gcmip = argv[1];
        try { LCM = mtsManagerLocal::GetInstance( gcmip, processname ); }
        catch(...) {
          std::cerr << "Failed to get GCM instance." << std::endl;
          return -1;
        }
    }else{
        LCM = mtsTaskManager::GetInstance();
    }


    //---------  Create all components -------------

    // ros wrapper
    mtsROSBridge JointControlBridge("JointControlBridge", 20 * cmn_ms);
    JointControlBridge.AddTopicFromReadFunction<prmPositionJointGet, sensor_msgs::JointState>(
                "PSM1", "GetPositionJoint", "/psm1/joint_position");
    LCM->AddComponent(&JointControlBridge);


    //------------- Connect components -------------------

#if 1
    // connect dv_joint_conrol to dv psm io
    if ( !LCM->Connect( processname, JointControlBridge.GetName(), "PSM1",
                                     "dvTeleop", "pid-slave", "Controller") )
    {
        std::cerr << "Failed to connect "
                  << processname << ":" << JointControlBridge.GetName() << ":FIX_REQUIRED to "
                  << "Master:Robot:FIX_PROVIDED"
                  << std::endl;

        return -1;
    }
#endif

    //-------------- create the components ------------------
    LCM->CreateAll();
    LCM->WaitForStateAll(mtsComponentState::READY, 20.0 * cmn_ms);

    // start the periodic Run
    LCM->StartAll();


    ROS_INFO("dv_joint_control Started\n");

    // run app
    cmnGetChar();

    ROS_INFO("dv_joint_control Finished\n");

    // cleanup
    LCM->KillAll();
    LCM->WaitForStateAll(mtsComponentState::FINISHED, 20.0 * cmn_ms);
    LCM->Cleanup();


    // stop all logs
    cmnLogger::SetMask(CMN_LOG_ALLOW_NONE);

    return 0;
}
