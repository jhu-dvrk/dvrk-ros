/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2015-05-23

  (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _dvrk_add_topics_functions_h
#define _dvrk_add_topics_functions_h

#include <cisst_ros_bridge/mtsROSBridge.h>

namespace dvrk {

    void add_topics_footpedal(mtsROSBridge & bridge,
                              const std::string & ros_namespace);

    void connect_bridge_footpedal(mtsROSBridge & bridge,
                                  const std::string & io_component_name);

    void add_topics_arm(mtsROSBridge & bridge,
                        const std::string & ros_namespace,
                        const std::string & arm_component_name);

    void add_topics_mtm(mtsROSBridge & bridge,
                        const std::string & ros_namespace,
                        const std::string & mtm_component_name);

    void add_topics_psm(mtsROSBridge & bridge,
                        const std::string & ros_namespace,
                        const std::string & psm_component_name);

    void add_topics_ecm(mtsROSBridge & bridge,
                        const std::string & ros_namespace,
                        const std::string & ecm_component_name);
}

#endif // _dvrk_add_topics_functions_h
