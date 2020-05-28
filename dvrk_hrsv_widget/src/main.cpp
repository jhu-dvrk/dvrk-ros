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

#include <hrsv_widget.h>

#include <signal.h>
#include <QApplication>
#include <ros/ros.h>

void signalHandler(int)
{
    QCoreApplication::exit(0);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hrsv_widget");
    QApplication application(argc, argv);

    ros::NodeHandle nodeHandle;
    hrsv_widget leftWidget(&nodeHandle);
    hrsv_widget rightWidget(&nodeHandle);

    leftWidget.show();
    rightWidget.show();

    signal(SIGINT, signalHandler);
    return application.exec();
}
