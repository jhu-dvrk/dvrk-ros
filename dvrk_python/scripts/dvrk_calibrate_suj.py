#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2024-01-09

# (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import crtk
import numpy
import math

import sys
import datetime

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QPushButton, QMessageBox
from PyQt5.QtCore import QTimer

class suj(object):
    """Simple arm API wrapping around ROS messages
    """

    class __Arm:

        class __Voltages:
            def __init__(self, ral, expected_interval):
                self.__crtk_utils = crtk.utils(self, ral, expected_interval)
                self.__crtk_utils.add_measured_js()

        def __init__(self, ral, expected_interval):
            self.__ral = ral
            self.primary_voltage = self.__Voltages(ral.create_child('/primary_voltage'), expected_interval)
            self.secondary_voltage = self.__Voltages(ral.create_child('/secondary_voltage'), expected_interval)

        def ral(self):
            return self.__ral

    # initialize the all SUJ arms
    def __init__(self, ral, expected_interval = 1.0):
        """Constructor.  This initializes a few data members and creates
        instances of classes for each SUJ arm."""
        self.__ral = ral.create_child('SUJ')
        self.__crtk_utils = crtk.utils(self, ral, expected_interval)
        for arm in ('ECM', 'PSM1', 'PSM2', 'PSM3'):
            setattr(self, arm, self.__Arm(self.__ral.create_child(arm), expected_interval))

    def ral(self):
        return self.__ral

class voltage(object):

    def __init__(self, name, sub_name, measured_jp, nb_joints):
        self.name = name
        self.sub_name = sub_name
        self.measured_jp = measured_jp
        self.nb_joints = nb_joints
        self.minimum = numpy.full([nb_joints], math.inf, dtype = float)
        self.maximum = numpy.full([nb_joints], -math.inf, dtype = float)

class main_widget(QWidget):

    def __init__(self, app, parent = None):
        super(main_widget, self).__init__(parent)
        self.app = app
        self.ral = crtk.ral('dvrk_calibrate_suj')
        self.ral.spin()

        self.SUJ = suj(self.ral)
        self.SUJ.ral().check_connections()

        self.arm_list = ['ECM', 'PSM1', 'PSM2', 'PSM3']
        self.pot_list = ['primary', 'secondary']

        # SUJ Si
        self.nb_joints = {
            'ECM': 4,
            'PSM1': 4,
            'PSM2': 4,
            'PSM3': 5
            }

        # setup CRTK clients
        self.all_voltages = {}
        for a in self.arm_list:
            for p in self.pot_list:
                self.all_voltages[a + '-' + p] = voltage(a, p,
                                                          eval('self.SUJ.' + a + '.' + p + '_voltage.measured_jp'),
                                                          self.nb_joints[a])

        # initialize mechanical limits for SI SUJ
        torad = math.pi / 180.0
        self.joint_limits = {}
        self.joint_limits['ECM-minimum']  = numpy.array([0.0,   -100.0 * torad, -157.0 * torad, -106.0 * torad])
        self.joint_limits['ECM-maximum']  = numpy.array([0.4826, 100.0 * torad,  157.0 * torad,  106.0 * torad])
        self.joint_limits['PSM1-minimum'] = numpy.array([0.0,    -95.0 * torad, -163.0 * torad,  -99.5 * torad])
        self.joint_limits['PSM1-maximum'] = numpy.array([0.4826,  95.0 * torad,    3.0 * torad,   95.0 * torad])
        self.joint_limits['PSM2-minimum'] = numpy.array([0.0,    -95.0 * torad,   -3.0 * torad,  -95.0 * torad])
        self.joint_limits['PSM2-maximum'] = numpy.array([0.4826,  95.0 * torad,  163.0 * torad,   99.5 * torad])
        self.joint_limits['PSM3-minimum'] = numpy.array([0.0,    -97.5 * torad, -154.5 * torad, -135.0 * torad, -30.0 * torad])
        self.joint_limits['PSM3-maximum'] = numpy.array([0.5842,  97.5 * torad,   82.0 * torad,  152.5 * torad,  95.0 * torad])
        self.pot_directions = {}
        self.pot_directions['ECM']  = numpy.array([-1.0,  1.0,  1.0,  1.0])
        self.pot_directions['PSM1'] = numpy.array([-1.0,  1.0,  1.0, -1.0])
        self.pot_directions['PSM2'] = numpy.array([-1.0,  1.0,  1.0, -1.0])
        self.pot_directions['PSM3'] = numpy.array([-1.0,  1.0, -1.0,  1.0, -1.0])

        # GUI
        self.setWindowTitle('dVRK SUJ Calibration')
        self.mainLayout = QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.table = QTableWidget(8, 7)
        self.mainLayout.addWidget(self.table)
        counter = 0
        for v in self.all_voltages.values():
            newItem = QTableWidgetItem(v.name)
            self.table.setItem(counter, 0, newItem)
            newItem = QTableWidgetItem(v.sub_name)
            self.table.setItem(counter, 1, newItem)
            for i in range(v.nb_joints):
                newItem = QTableWidgetItem(f'[{v.minimum[i]}, {v.maximum[i]}]')
                self.table.setItem(counter, 2 + i, newItem)
            counter += 1

        self.showButton = QPushButton('Show')
        self.mainLayout.addWidget(self.showButton)
        self.showButton.clicked.connect(self.show_cb)

        self.showButton = QPushButton('Quit')
        self.mainLayout.addWidget(self.showButton)
        self.showButton.clicked.connect(self.quit_cb)

        # timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.timer_cb)
        self.timer.start(20) # in ms

    def timer_cb(self):
        counter = 0
        for v in self.all_voltages.values():
            jp = v.measured_jp()
            v.minimum = numpy.minimum(v.minimum, jp)
            v.maximum = numpy.maximum(v.maximum, jp)
            for i in range(v.nb_joints):
                item = self.table.item(counter, 2 + i)
                item.setText(f'[{v.minimum[i]}, {v.maximum[i]}] -> {v.maximum[i] - v.minimum[i]}')
            counter += 1
        self.table.resizeColumnsToContents()


    def show_cb(self):
        self.all_offsets = {}
        self.all_scales = {}
        now = datetime.datetime.now()
        print(f'-- at {now.hour}:{now.minute}:{now.second} ---')
        for a in self.arm_list:
            print(f'--- {a} ----')
            for p in self.pot_list:
                self.all_scales[a + p] = numpy.zeros(self.nb_joints[a])
                self.all_offsets[a + p] = numpy.zeros(self.nb_joints[a])
                for j in range(self.nb_joints[a]):
                    j_min = self.joint_limits[a + '-minimum'][j]
                    j_max = self.joint_limits[a + '-maximum'][j]
                    v_min = self.all_voltages[a + '-' + p].minimum[j]
                    v_max = self.all_voltages[a + '-' + p].maximum[j]
                    if self.pot_directions[a][j] > 0:
                        s = ((j_max - j_min) / (v_max - v_min))
                        o = j_min - (s * v_min)
                    else:
                        s = ((j_min - j_max) / (v_max - v_min))
                        o = j_max - (s * v_min)
                    self.all_scales[a + p][j] = s
                    self.all_offsets[a + p][j] = o
                print(f'"{p}-offsets": {self.all_offsets[a + p].tolist()},')
                print(f'"{p}-scales": {self.all_scales[a + p].tolist()},')

    def quit_cb(self):
        msg = QMessageBox()
        msg.setText("Quit?")
        msg.setWindowTitle("dvrk_calibrate_suj quit")
        msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        result = msg.exec()
        if result == QMessageBox.Ok:
            self.ral.shutdown()
            self.app.quit()

    def closeEvent(self, event):
        self.ral.shutdown()


# GUI
app = QApplication([])
widget = main_widget(app)
widget.show()

app.exec()
widget.timer.stop()
