#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2023-09-12

# (C) Copyright 2023 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a console with at least one PSM teleoperation component
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To reset the arms in the
# > rosrun dvrk_python dvrk_reset_teleoperation -m <mtm-name> -p <psm-name>

"""
Reposition MTMs and PSMs for teleoperation.  You can specify one
or more MTMs and PSMs.  Examples:
> dvrk_reset_teleoperation -m MTMR -p PSM1
> dvrk_reset_teleoperation -m MTMR MTML -p PSM1 PSM2 PSM3
> dvrk_reset_teleoperation -m MTMR -p PSM1 -m MTML -p PSM2 PSM3
"""

import sys
import argparse
import crtk
import dvrk
import numpy
import PyKDL

# example of application using arm.py
class reset_teleoperation:

    # configuration
    def __init__(self, ral, mtms, psms, expected_interval):
        print('dvrk_reset_teleoperation for MTM(s): {} and PSM(s): {}'.format(mtms, psms))
        self.ral = ral
        self.expected_interval = expected_interval
        self.mtms = []
        for mtm in mtms:
            self.mtms.append(dvrk.mtm(ral = ral,
                                      arm_name = mtm,
                                      expected_interval = expected_interval))
        self.psms = []
        for psm in psms:
            self.psms.append(dvrk.psm(ral = ral,
                                      arm_name = psm,
                                      expected_interval = expected_interval))
        self.console = dvrk.console(ral = ral)

    # run
    def run(self):
        # check all connections at once
        self.ral.check_connections()

        # disable teleoperation
        self.console.teleop_stop()

        # send move to the all arms
        move_handles = []
        for mtm in self.mtms:
            move_handles.append(mtm.move_jp(numpy.zeros(7)))
        for psm in self.psms:
            psm_goal_jp =  numpy.copy(psm.setpoint_jp())
            psm_goal_jp[3] = 0.0
            move_handles.append(psm.move_jp(psm_goal_jp))

        # wait for all completed
        for handle in move_handles:
            handle.wait()

        # restart teleop
        self.console.teleop_start()
        print('done')


if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser(description = __doc__,
                                     formatter_class = argparse.RawTextHelpFormatter)
    parser.add_argument('-m', '--mtms', type = str, required = True,
                        action = 'extend', nargs = '+',
                        choices = ['MTML', 'MTMR'],
                        help = 'MTM arm name that needs to be reset')
    parser.add_argument('-p', '--psms', type = str,
                        action = 'extend', nargs = '+',
                        choices = ['PSM1', 'PSM2', 'PSM3'],
                        help = 'PSM arm name that needs to be reset')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_reset_teleoperation')
    application = reset_teleoperation(ral, args.mtms, args.psms, args.interval)
    ral.spin_and_execute(application.run)
