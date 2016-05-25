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
#include <cisstVector/vctQtWidgetFrame4x4.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>

#include <sawROS/mtsROSToCISST.h>
#include <sawROS/mtsCISSTToROS.h>

#include <QtGui>
#include <QObject>

// set up joint state variables

class RobotQWidget: public QWidget
{
    Q_OBJECT
public:
    RobotQWidget(const std::string &name, const double &period):
        QWidget(0, 0),
        counter_(0)
    {
        // subscriber
        // NOTE: queue size is set to 1 to make sure data is fresh
        sub_mtm_pose_ = nh_.subscribe("/dvrk_mtm/cartesian_pose_current", 1,
                                        &RobotQWidget::master_pose_cb, this);
        sub_psm_pose_ = nh_.subscribe("/dvrk_psm/cartesian_pose_current", 1,
                                       &RobotQWidget::slave_pose_cb, this);

        mtm_pose_qt_ = new vctQtWidgetFrame4x4DoubleRead;
        psm_pose_qt_ = new vctQtWidgetFrame4x4DoubleRead;
        QVBoxLayout *mainLayout = new QVBoxLayout;
        mainLayout->addWidget(mtm_pose_qt_);
        mainLayout->addWidget(psm_pose_qt_);
        this->setLayout(mainLayout);

        // show widget & start timer
        this->show();
        startTimer(period);  // 50 ms
    }

    ~RobotQWidget(){}

    void timerEvent(QTimerEvent *)
    {
        // refresh data
        ros::spinOnce();

        mtm_pose_qt_->SetValue(mtm_pose_cur_);
        psm_pose_qt_->SetValue(psm_pose_cur_);

#if 1
        if (counter_%20 == 0) {
            std::cerr << " mtm = " << std::endl
                      << mtm_pose_cur_ << std::endl;
            std::cerr << "psm = " << std::endl
                      << psm_pose_cur_ << std::endl << std::endl;
        }
#endif
        counter_++;
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



int main(int argc, char** argv)
{
    // ros initialization
    ros::init(argc, argv, "dvrk_teleop");

    QApplication qapp(argc, argv);
    RobotQWidget gui("gui", 50);
    qapp.exec();

    cmnLogger::Kill();

    return 0;
}
