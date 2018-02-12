/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-05-23

  (C) Copyright 2015-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _dvrk_console_h
#define _dvrk_console_h

#include <dvrk_utilities/dvrk_add_topics_functions.h>

class mtsIntuitiveResearchKitConsole;

namespace dvrk {
    class console
    {
    public:
        console(const double & publish_rate_in_seconds,
                const double & tf_rate_in_seconds,
                const std::string & ros_namespace,
                mtsIntuitiveResearchKitConsole * mts_console,
                const dvrk_topics_version::version version);
        void Configure(const std::string & jsonFile);
        void Connect(void);
    protected:
        std::string mBridgeName;
        std::string mTfBridgeName;
        std::string mNameSpace;
        mtsIntuitiveResearchKitConsole * mConsole;
        dvrk_topics_version::version mVersion;
        std::list<std::string> mIOInterfaces;
    };
}

#endif // _dvrk_console_h
