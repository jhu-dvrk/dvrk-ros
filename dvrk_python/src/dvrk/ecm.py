#  Author(s):  Anton Deguet
#  Created on: 2016-05

#   (C) Copyright 2016-2017 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

from dvrk.arm import *

class ecm(arm):
    """Simple robot API wrapping around ROS messages
    """
    # initialize the robot
    def __init__(self, ecm_name, ros_namespace = '/dvrk/'):
        # first call base class constructor
        self._arm__init_arm(ecm_name, ros_namespace)


    def insert_endoscope(self, depth, interpolate = True, blocking = True):
        "insert the endoscope, by moving it to an absolute depth"
        return self.move_joint_one(depth, 2, interpolate, blocking)


    def dinsert_endoscope(self, depth, interpolate = True, blocking = True):
        "insert the endoscope, by moving it an additional depth"
        return self.dmove_joint_one(depth, 2, interpolate, blocking)
