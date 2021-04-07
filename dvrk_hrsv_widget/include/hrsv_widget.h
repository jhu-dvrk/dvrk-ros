/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-03-13

  (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _hrsv_widget_h
#define _hrsv_widget_h

#include <QWidget>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <diagnostic_msgs/KeyValue.h>

class QLabel;

class hrsv_widget: public QWidget
{
    Q_OBJECT;

public:
    hrsv_widget(ros::NodeHandle * nodeHandle);
    ~hrsv_widget();

    void setupUi(void);
    void setupROS(void);

private slots:
    void timerEvent(QTimerEvent *);

protected:
    ros::NodeHandle * mNodeHandle;

    QLabel * mLeftLabel;
    QLabel * mRightLabel;
    QLabel * mMainLabel;

    struct {
        std::string PSM;
        std::string Status;
    } mLeft, mRight;

    bool mClutch = false;
    bool mOperatorPresent = false;
    std::string mCameraState;

    ros::Subscriber mClutchSubscriber;
    void ClutchCallback(const sensor_msgs::Joy & message);

    ros::Subscriber mOperatorPresentSubscriber;
    void OperatorPresentCallback(const sensor_msgs::Joy & message);

    ros::Subscriber mCameraStateSubscriber;
    void CameraStateCallback(const std_msgs::String & message);

    ros::Subscriber mPSMSelectedCallback;
    void PSMSelectedCallback(const diagnostic_msgs::KeyValue & message);

    // all possible callbacks
    ros::Subscriber
        mMTMRPSM1Subscriber,
        mMTMRPSM2Subscriber,
        mMTMRPSM3Subscriber,
        mMTMLPSM1Subscriber,
        mMTMLPSM2Subscriber,
        mMTMLPSM3Subscriber;
    void MTMRPSMCallback(const std_msgs::String & message);
    void MTMLPSMCallback(const std_msgs::String & message);

    void UpdateLabels(void);
};

#endif // _hrsv_widget_h
