/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-05-23

  (C) Copyright 2015-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _dvrk_console_h
#define _dvrk_console_h

#include <cisst_ros_crtk/mts_ros_crtk_bridge.h>
#include <dvrk_utilities/dvrk_add_topics_functions.h>

class mtsIntuitiveResearchKitConsole;

namespace dvrk {
    class console: public mts_ros_crtk_bridge
    {
    public:
        console(const std::string & name,
                ros::NodeHandle * node_handle,
                const double & publish_rate_in_seconds,
                const double & tf_rate_in_seconds,
                mtsIntuitiveResearchKitConsole * mts_console);
        void Configure(const std::string & jsonFile);
        void Connect(void) override;

        void bridge_interface_provided_arm(const std::string & _component_name,
                                           const std::string & _interface_name,
                                           const std::string & _ros_namespace,
                                           const double _publish_period_in_seconds,
                                           const double _tf_period_in_seconds);

        void bridge_interface_provided_ecm(const std::string & _component_name,
                                           const std::string & _interface_name,
                                           const std::string & _ros_namespace,
                                           const double _publish_period_in_seconds,
                                           const double _tf_period_in_seconds);

        void bridge_interface_provided_mtm(const std::string & _component_name,
                                           const std::string & _interface_name,
                                           const std::string & _ros_namespace,
                                           const double _publish_period_in_seconds,
                                           const double _tf_period_in_seconds);

        void bridge_interface_provided_psm(const std::string & _component_name,
                                           const std::string & _interface_name,
                                           const std::string & _ros_namespace,
                                           const double _publish_period_in_seconds,
                                           const double _tf_period_in_seconds);
    protected:
        std::string mBridgeName;
        std::string mTfBridgeName;
        mtsIntuitiveResearchKitConsole * mConsole;
        std::list<std::string> mIOInterfaces;
    };
}

#endif // _dvrk_console_h
