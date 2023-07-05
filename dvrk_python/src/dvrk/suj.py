#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2021 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import crtk

class suj(object):
    """Simple arm API wrapping around ROS messages
    """

    # local kinematics
    class __Local:
        def __init__(self, ral, expected_interval):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval)
            self.__crtk_utils.add_measured_cp()

    # initialize the arm
    def __init__(self, ral, arm_name, expected_interval = 1.0):
        """Constructor.  This initializes a few data members.It
        requires an arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `SUJ/PSM1`"""
        self.suj_ral = ral.create_child('SUJ').create_child(arm_name)

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.suj_ral, expected_interval)

        # add crtk features that we need and are supported by the dVRK
        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_measured_js()
        self.__crtk_utils.add_measured_cp()
        self.__crtk_utils.add_move_jp()

        self.local = self.__Local(self.suj_ral.create_child('/local'), expected_interval)
