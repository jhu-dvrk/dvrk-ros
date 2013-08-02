#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

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

protected slots:
    // console
    void slot_homeButton_pressed(void);
    void slot_manualButton_pressed(void);
    void slot_teleopButton_pressed(void);

    // mtm
    void slot_clutchButton_pressed(bool state);
    void slot_headButton_pressed(bool state);

    // psm
    void slot_moveToolButton_pressed(bool state);

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

    ros::Publisher pub_mtm_control_mode_;
    ros::Publisher pub_psm_control_mode_;
    ros::Publisher pub_enable_slider_;

    std_msgs::Int8 msg_mtm_mode_;
    std_msgs::Int8 msg_psm_mode_;
};
