/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-01-19

  (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

// system
#include <iostream>
#include <map>

// cisst/saw
#include <cisst_ros_bridge/mtsROSBridge.h>

class dvrk_arm_bridge: public mtsROSBridge
{
public:
    dvrk_arm_bridge(const std::string & name,
                    const std::string & rosNamespace,
                    const double periodInSeconds,
                    bool spin = false,
                    bool sig = true,
                    ros::NodeHandle* nh = NULL);

protected:
    void setup(void);
    std::string ros_namespace;
};
