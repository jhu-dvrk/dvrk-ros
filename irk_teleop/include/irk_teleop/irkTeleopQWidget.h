#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>

#include <cisstRobot/robManipulator.h>
#include <cisstVector.h>
#include <cisstVector/vctQtWidgetFrame4x4.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>

#include <sawROS/mtsROSToCISST.h>
#include <sawROS/mtsCISSTToROS.h>

#include <QtGui>
#include <QObject>

// set up joint state variables

class irkTeleopQWidget: public QWidget
{
    Q_OBJECT
public:
    irkTeleopQWidget(const std::string &name, const double &period);
    ~irkTeleopQWidget(){}
    void timerEvent(QTimerEvent *);

protected:
    void master_pose_cb(const geometry_msgs::PoseConstPtr &msg);
    void slave_pose_cb(const geometry_msgs::PoseConstPtr &msg);

    vctFrm4x4 mtm_pose_cur_;
    vctFrm4x4 psm_pose_cur_;
    size_t counter_;

    // gui
    vctQtWidgetFrame4x4DoubleRead *mtm_pose_qt_;
    vctQtWidgetFrame4x4DoubleRead *psm_pose_qt_;

    // ros variables
    ros::NodeHandle nh_;

    ros::Subscriber sub_mtm_pose_;
    ros::Subscriber sub_psm_pose_;
};
