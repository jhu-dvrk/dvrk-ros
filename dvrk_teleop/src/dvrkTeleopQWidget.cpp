/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <ros/package.h>

#include "dvrk_teleop/dvrkTeleopQWidget.h"
#include "dvrk_kinematics/mtm_logic.h"
#include "dvrk_kinematics/psm_logic.h"

#include <QVBoxLayout>
#include <QButtonGroup>
#include <QApplication>

// set up joint state variables

dvrkTeleopQWidget::dvrkTeleopQWidget(const std::string &name, const double &period):
    QWidget(), counter_(0)
{
    is_head_in_ = false;
    is_clutched_ = false;
    is_move_psm_ = false;

    // subscriber
    // NOTE: queue size is set to 1 to make sure data is fresh
    sub_mtm_pose_ = nh_.subscribe("/dvrk_mtm/cartesian_pose_current", 1,
                                    &dvrkTeleopQWidget::master_pose_cb, this);
    sub_psm_pose_ = nh_.subscribe("/dvrk_psm/cartesian_pose_current", 1,
                                   &dvrkTeleopQWidget::slave_pose_cb, this);

    // publisher
    pub_teleop_enable_ = nh_.advertise<std_msgs::Bool>("/dvrk_teleop/enable", 100);
    pub_mtm_control_mode_ = nh_.advertise<std_msgs::Int8>("/dvrk_mtm/control_mode", 100);
    pub_psm_control_mode_ = nh_.advertise<std_msgs::Int8>("/dvrk_psm/control_mode", 100);
    pub_enable_slider_ = nh_.advertise<sensor_msgs::JointState>("/dvrk_mtm/joint_state_publisher/enable_slider", 100);

    // pose display
    mtm_pose_qt_ = new vctQtWidgetFrame4x4DoubleRead;
    psm_pose_qt_ = new vctQtWidgetFrame4x4DoubleRead;

    QVBoxLayout *poseLayout = new QVBoxLayout;
    poseLayout->addWidget(mtm_pose_qt_);
    poseLayout->addWidget(psm_pose_qt_);

    // common console
    QGroupBox *consoleBox = new QGroupBox("Console");
    QPushButton *consoleHomeButton = new QPushButton(tr("Home"));
    QPushButton *consoleManualButton = new QPushButton(tr("Manual"));
    QPushButton *consoleTeleopTestButton = new QPushButton(tr("TeleopTest"));
    consoleTeleopButton = new QPushButton(tr("Teleop"));
    consoleHomeButton->setStyleSheet("font: bold; color: green;");
    consoleManualButton->setStyleSheet("font: bold; color: red;");
    consoleTeleopTestButton->setStyleSheet("font: bold; color: blue;");
    consoleTeleopButton->setStyleSheet("font: bold; color: brown;");
    consoleHomeButton->setCheckable(true);
    consoleManualButton->setCheckable(true);
    consoleTeleopTestButton->setCheckable(true);
    consoleTeleopButton->setCheckable(true);

    QButtonGroup *consoleButtonGroup = new QButtonGroup;
    consoleButtonGroup->setExclusive(true);
    consoleButtonGroup->addButton(consoleHomeButton);
    consoleButtonGroup->addButton(consoleManualButton);
    consoleButtonGroup->addButton(consoleTeleopTestButton);
    consoleButtonGroup->addButton(consoleTeleopButton);

    QHBoxLayout *consoleBoxLayout = new QHBoxLayout;
    consoleBoxLayout->addWidget(consoleHomeButton);
    consoleBoxLayout->addWidget(consoleManualButton);
    consoleBoxLayout->addWidget(consoleTeleopTestButton);
    consoleBoxLayout->addWidget(consoleTeleopButton);
    consoleBoxLayout->addStretch();
    consoleBox->setLayout(consoleBoxLayout);

    // mtm console
    mtmBox = new QGroupBox("MTM");
    QVBoxLayout *mtmBoxLayout = new QVBoxLayout;
    mtmClutchButton = new QPushButton(tr("Clutch"));
    mtmClutchButton->setCheckable(true);
    mtmBoxLayout->addWidget(mtmClutchButton);

    mtmHeadButton = new QPushButton(tr("Head"));
    mtmHeadButton->setCheckable(true);
    mtmBoxLayout->addWidget(mtmHeadButton);

    mtmBoxLayout->addStretch();
    mtmBox->setLayout(mtmBoxLayout);

    // psm console
    psmBox = new QGroupBox("PSM");
    QVBoxLayout *psmBoxLayout = new QVBoxLayout;
    psmMoveButton = new QPushButton(tr("Move Tool"));
    psmMoveButton->setCheckable(true);
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
    std::string filename = ros::package::getPath("dvrk_teleop");
    filename.append("/src/default.css");
    QFile defaultStyleFile(filename.c_str());
    defaultStyleFile.open(QFile::ReadOnly);
    QString styleSheet = QLatin1String(defaultStyleFile.readAll());
    this->setStyleSheet(styleSheet);

    // now connect everything
    connect(consoleHomeButton, SIGNAL(clicked()), this, SLOT(slot_homeButton_pressed()));
    connect(consoleManualButton, SIGNAL(clicked()), this, SLOT(slot_manualButton_pressed()));
    connect(consoleTeleopTestButton, SIGNAL(clicked()), this, SLOT(slot_teleopTestButton_pressed()));
    connect(consoleTeleopButton, SIGNAL(toggled(bool)), this, SLOT(slot_teleopButton_toggled(bool)));

    connect(mtmHeadButton, SIGNAL(clicked(bool)), this, SLOT(slot_headButton_pressed(bool)));
    connect(mtmClutchButton, SIGNAL(clicked(bool)), this, SLOT(slot_clutchButton_pressed(bool)));
    connect(psmMoveButton, SIGNAL(clicked(bool)), this, SLOT(slot_moveToolButton_pressed(bool)));

    // show widget & start timer
    startTimer(period);  // 50 ms
    slot_teleopButton_toggled(false);
}


void dvrkTeleopQWidget::timerEvent(QTimerEvent *)
{
    // check ros::ok()
    if (!ros::ok()) {
        QApplication::quit();
    }

    // refresh data
    ros::spinOnce();

    mtm_pose_qt_->SetValue(mtm_pose_cur_);
    psm_pose_qt_->SetValue(psm_pose_cur_);

    counter_++;
}


void dvrkTeleopQWidget::master_pose_cb(const geometry_msgs::PoseConstPtr &msg)
{
    mtsROSToCISST((*msg), mtm_pose_cur_);
}

void dvrkTeleopQWidget::slave_pose_cb(const geometry_msgs::PoseConstPtr &msg)
{
    mtsROSToCISST((*msg), psm_pose_cur_);
}

void dvrkTeleopQWidget::set_master_slave_state()
{
    if (consoleTeleopButton->isChecked()) {
        // check teleop enable status
        if (is_head_in_) {
            if (!is_clutched_ && !is_move_psm_) {
                // enable teleop node
                msg_teleop_enable.data = true;
                msg_mtm_mode_.data = MTM::MODE_TELEOP;
                msg_psm_mode_.data = PSM::MODE_TELEOP;
            } else if (is_clutched_) {
                // mtm move, keep orientation as psm
                msg_teleop_enable.data = false;
                msg_mtm_mode_.data = MTM::MODE_CLUTCH;
                msg_psm_mode_.data = PSM::MODE_HOLD;
            } else {
                msg_teleop_enable.data = false;
                msg_mtm_mode_.data = MTM::MODE_HOLD;
                msg_psm_mode_.data = PSM::MODE_HOLD;
            }
        } else {
            msg_teleop_enable.data = false;
            if (is_clutched_) {
                msg_mtm_mode_.data = MTM::MODE_CLUTCH;
            } else {
                msg_mtm_mode_.data = MTM::MODE_HOLD;
            }

            if (is_move_psm_) {
                msg_psm_mode_.data = PSM::MODE_MANUAL;
            } else {
                msg_psm_mode_.data = PSM::MODE_HOLD;
            }
        }

        pub_teleop_enable_.publish(msg_teleop_enable);
        pub_mtm_control_mode_.publish(msg_mtm_mode_);
        pub_psm_control_mode_.publish(msg_psm_mode_);
    }
}


// -------- slot -----------
void dvrkTeleopQWidget::slot_homeButton_pressed(void)
{
    msg_mtm_mode_.data = MTM::MODE_RESET;
    msg_psm_mode_.data = PSM::MODE_RESET;
    pub_mtm_control_mode_.publish(msg_mtm_mode_);
    pub_psm_control_mode_.publish(msg_psm_mode_);
}

void dvrkTeleopQWidget::slot_manualButton_pressed(void)
{
    msg_mtm_mode_.data = MTM::MODE_MANUAL;
    msg_psm_mode_.data = PSM::MODE_MANUAL;
    pub_mtm_control_mode_.publish(msg_mtm_mode_);
    pub_psm_control_mode_.publish(msg_psm_mode_);
}

void dvrkTeleopQWidget::slot_teleopTestButton_pressed(void)
{
    msg_mtm_mode_.data = MTM::MODE_TELEOP;
    msg_psm_mode_.data = PSM::MODE_TELEOP;
    pub_mtm_control_mode_.publish(msg_mtm_mode_);
    pub_psm_control_mode_.publish(msg_psm_mode_);
}

void dvrkTeleopQWidget::slot_teleopButton_toggled(bool state)
{
    if (state) {
        msg_mtm_mode_.data = MTM::MODE_HOLD;
        msg_psm_mode_.data = PSM::MODE_HOLD;
        pub_mtm_control_mode_.publish(msg_mtm_mode_);
        pub_psm_control_mode_.publish(msg_psm_mode_);

        // enable boxes
        mtmBox->setEnabled(true);
        psmBox->setEnabled(true);
    } else {
        mtmBox->setEnabled(false);
        psmBox->setEnabled(false);
    }
}

void dvrkTeleopQWidget::slot_clutchButton_pressed(bool state)
{
    is_clutched_ = state;
    set_master_slave_state();
}

void dvrkTeleopQWidget::slot_headButton_pressed(bool state)
{
    is_head_in_ = state;
    set_master_slave_state();
}


void dvrkTeleopQWidget::slot_moveToolButton_pressed(bool state)
{
    is_move_psm_ = state;
    set_master_slave_state();
}
