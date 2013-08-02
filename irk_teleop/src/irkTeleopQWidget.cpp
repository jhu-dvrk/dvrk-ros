#include <ros/package.h>
#include <std_msgs/Int8.h>

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

    pub_control_mode_ = nh_.advertise<std_msgs::Int8>("/irk/control_mode", 100);
    pub_enable_slider_ = nh_.advertise<sensor_msgs::JointState>("/irk_mtm/joint_state_publisher/enable_slider", 100);


    // pose display
    mtm_pose_qt_ = new vctQtWidgetFrame4x4DoubleRead;
    psm_pose_qt_ = new vctQtWidgetFrame4x4DoubleRead;
//    mtm_pose_qt_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    psm_pose_qt_->resize(200, 200);

    QVBoxLayout *poseLayout = new QVBoxLayout;
    poseLayout->addWidget(mtm_pose_qt_);
    poseLayout->addWidget(psm_pose_qt_);

    // common console
    QGroupBox *consoleBox = new QGroupBox("Console");
    QHBoxLayout *consoleBoxLayout = new QHBoxLayout;
    QPushButton *consoleHomeButton = new QPushButton(tr("Home"));
    QPushButton *consoleManualButton = new QPushButton(tr("Manual"));
    QPushButton *consoleTeleopButton = new QPushButton(tr("Teleop"));
    consoleHomeButton->setStyleSheet("background: green; font: bold; color: yellow;");
    consoleBoxLayout->addWidget(consoleHomeButton);
    consoleBoxLayout->addWidget(consoleManualButton);
    consoleBoxLayout->addWidget(consoleTeleopButton);
    consoleBoxLayout->addStretch();
    consoleBox->setLayout(consoleBoxLayout);

    // mtm console
    QGroupBox *mtmBox = new QGroupBox("MTM");
    QVBoxLayout *mtmBoxLayout = new QVBoxLayout;
    QPushButton *mtmClutchButton = new QPushButton(tr("Clutch"));
    mtmClutchButton->setCheckable(true);
    mtmBoxLayout->addWidget(mtmClutchButton);

    QPushButton *mtmHeadButton = new QPushButton(tr("Head"));
    mtmHeadButton->setCheckable(true);
    mtmBoxLayout->addWidget(mtmHeadButton);

    mtmBoxLayout->addStretch();
    mtmBox->setLayout(mtmBoxLayout);

    // psm console
    QGroupBox *psmBox = new QGroupBox("PSM");
    QVBoxLayout *psmBoxLayout = new QVBoxLayout;
    QPushButton *psmMoveButton = new QPushButton(tr("Move Tool"));
    psmBoxLayout->addWidget(psmMoveButton);
    psmBoxLayout->addStretch();
    psmBox->setLayout(psmBoxLayout);

    // rightLayout
    QGridLayout *rightLayout = new QGridLayout;
    rightLayout->addWidget(consoleBox, 0, 0, 1, 2);
    rightLayout->addWidget(mtmBox, 1, 0);
    rightLayout->addWidget(psmBox, 1, 1);

    QHBoxLayout *mainLayout = new QHBoxLayout;
    mainLayout->addLayout(poseLayout);
    mainLayout->addLayout(rightLayout);

    this->setLayout(mainLayout);
    this->resize(sizeHint());
    this->setWindowTitle("Teleopo GUI");

    // set stylesheet
    std::string filename = ros::package::getPath("irk_teleop");
    filename.append("/src/default.css");
    QFile defaultStyleFile(filename.c_str());
    defaultStyleFile.open(QFile::ReadOnly);
    QString styleSheet = QLatin1String(defaultStyleFile.readAll());
    this->setStyleSheet(styleSheet);

    // now connect everything
    connect(consoleHomeButton, SIGNAL(clicked()), this, SLOT(slot_homeButton_pressed()));
    connect(consoleManualButton, SIGNAL(clicked()), this, SLOT(slot_manualButton_pressed()));
    connect(consoleTeleopButton, SIGNAL(clicked()), this, SLOT(slot_teleopButton_pressed()));

    connect(mtmHeadButton, SIGNAL(clicked(bool)), this, SLOT(slot_headButton_pressed(bool)));


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


// -------- slot -----------
void irkTeleopQWidget::slot_homeButton_pressed(void)
{
    std_msgs::Int8 msg_mode;
    msg_mode.data = 0;  // 0 is HOME
    pub_control_mode_.publish(msg_mode);
}

void irkTeleopQWidget::slot_manualButton_pressed(void)
{
    std_msgs::Int8 msg_mode;
    msg_mode.data = 1;  // 1 is MANUAL
    pub_control_mode_.publish(msg_mode);
}

void irkTeleopQWidget::slot_teleopButton_pressed(void)
{
    std_msgs::Int8 msg_mode;
    msg_mode.data = 2;  // 2 is TELEOP
    pub_control_mode_.publish(msg_mode);
}

void irkTeleopQWidget::slot_headButton_pressed(bool state)
{
    QPushButton *headButton = (QPushButton*)sender();

    sensor_msgs::JointState msg_js;
    msg_js.name.clear();
    msg_js.name.push_back("right_outer_yaw_joint");
    msg_js.name.push_back("right_shoulder_pitch_joint");
    msg_js.name.push_back("right_elbow_pitch_joint");
    msg_js.name.push_back("right_wrist_platform_joint");
    msg_js.name.push_back("right_wrist_pitch_joint");
    msg_js.name.push_back("right_wrist_yaw_joint");
    msg_js.name.push_back("right_wrist_roll_joint");

    msg_js.position.resize(7);
    msg_js.position[0] = 1;
    msg_js.position[1] = 1;
    msg_js.position[2] = 1;
    msg_js.position[3] = 1;
    msg_js.position[4] = 1;
    msg_js.position[5] = 1;
    msg_js.position[6] = 1;

    if (state) {
        pub_enable_slider_.publish(msg_js);
    } else {
        msg_js.position[3] = -1;
        msg_js.position[4] = -1;
        msg_js.position[5] = -1;
        msg_js.position[6] = -1;
        pub_enable_slider_.publish(msg_js);
    }
}



















