/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2015-05-23

  (C) Copyright 2015-2020 Johns Hopkins University (JHU), All Rights Reserved.

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

    /*! This method adds topics from the IO level for the whole
      system. */
    void add_topics_io(mtsROSBridge & bridge,
                       const std::string & ros_namespace);

    /*! This method connects all the required interfaces for IOs, it
      must be used after add_topics_io. */
    void connect_bridge_io(const std::string & bridge_name,
                           const std::string & io_component_name);

    /*! This method adds topics from the IO level for a given arm.
      All the topics are publishers only, i.e. we do not allow direct
      control of the IO.  All commands sent to the dVRK must use the
      MTM, ECM or PSM topics to ensure that the system is properly
      initialized.  The topics added are mostly useful for low level
      data collection and debugging. */
    void add_topics_io(mtsROSBridge & bridge,
                       const std::string & ros_namespace,
                       const std::string & arm_name);

    /*! This method connects all the required interfaces for an arm
      IOs, it must be used after add_topics_io. */
    void connect_bridge_io(const std::string & bridge_name,
                           const std::string & io_component_name,
                           const std::string & arm_name);
}

#endif // _dvrk_add_topics_functions_h
