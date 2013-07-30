
#include "irk_teleop/irkTeleopQWidget.h"

// set up joint state variables

irkTeleopQWidget::irkTeleopQWidget(const std::string &name, const double &period):
    QWidget(), counter_(0)
{
    // subscriber
    // NOTE: queue size is set to 1 to make sure data is fresh
    sub_mtm_pose_ = nh_.subscribe("/irk_mtm/cartesian_pose_current", 1,
                                    &irkTeleopQWidget::master_pose_cb, this);
    sub_psm_pose_ = nh_.subscribe("/irk_psm/cartesian_pose_current", 1,
                                   &irkTeleopQWidget::slave_pose_cb, this);

    mtm_pose_qt_ = new vctQtWidgetFrame4x4DoubleRead;
    psm_pose_qt_ = new vctQtWidgetFrame4x4DoubleRead;
//    mtm_pose_qt_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    psm_pose_qt_->resize(200, 200);
    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(mtm_pose_qt_);
    mainLayout->addWidget(psm_pose_qt_);
    this->setLayout(mainLayout);
    this->resize(sizeHint());
    this->setWindowTitle("Teleopo GUI");

    // show widget & start timer
    startTimer(period);  // 50 ms
}


void irkTeleopQWidget::timerEvent(QTimerEvent *)
{
    // refresh data
    ros::spinOnce();

    mtm_pose_qt_->SetValue(mtm_pose_cur_);
    psm_pose_qt_->SetValue(psm_pose_cur_);

#if 0
    if (counter_%20 == 0) {
        std::cerr << " mtm = " << std::endl
                  << mtm_pose_cur_ << std::endl;
        std::cerr << "psm = " << std::endl
                  << psm_pose_cur_ << std::endl << std::endl;
    }
#endif
    counter_++;
}


void irkTeleopQWidget::master_pose_cb(const geometry_msgs::PoseConstPtr &msg)
{
    mtsROSToCISST((*msg), mtm_pose_cur_);
}

void irkTeleopQWidget::slave_pose_cb(const geometry_msgs::PoseConstPtr &msg)
{
    mtsROSToCISST((*msg), psm_pose_cur_);
}
