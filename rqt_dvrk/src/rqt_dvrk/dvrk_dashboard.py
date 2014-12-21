# Zihan Chen
# 2014-12-19
# NOTE: part of the code is from rqt_dvrk
# https://github.com/jhu-lcsr/orocos_dvrk

import rospy
from math import pi

from qt_gui.plugin import Plugin
from python_qt_binding import QtGui
from QtGui import QColor
from python_qt_binding import QtCore
from python_qt_binding.Qwt.Qwt import QwtThermo
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# from urdf_parser_py.urdf import URDF


# TODO
#  - get joint torque feedback
#    - publish current jnt torque o
#    - check if all jnt torque limit are set
#  - joint limit database
#  - get num of jnts from urdf
#  - add support for ECM
#  - use actionlib instead of just publishing msgs
#  - add power status stuff

class dvrkDashboard(Plugin):
    def __init__(self, context):
        super(dvrkDashboard, self).__init__(context)

        # give QObjects reasonable names
        self.setObjectName('dvrkDashboard')

        # process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # create qwidget
        self._widget = QtGui.QWidget()
        self._widget.setObjectName('dvrkDashboardUI')

        # serial number
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # add widget to the user interface
        context.add_widget(self._widget)
        self.context = context

        # ---- Get Widget -----
        self.init_ui()

        # ---- States -----
        self.namespace = 'dvrk_mtmr'
        self.jnt_pos = []
        self.jnt_eff = []
        self.init_ros()

        # ---- Timer -----
        self.update_timer = QtCore.QTimer(self)
        self.update_timer.setInterval(50)
        self.update_timer.timeout.connect(self.update_widget_values)
        self.update_timer.start()
        pass

    def init_ui(self):
        # type combobox
        ui_type_lable = QtGui.QLabel('Robot')
        ui_type_lable.setStyleSheet('QLabel {font: bold}')
        ui_type_lable.setMaximumWidth(70)
        # ui_type_lable.setFont(QtGui.QFont(QtGui.QFont.Bold))
        ui_type_lable.setAlignment(QtCore.Qt.AlignCenter)
        self.ui_type = QtGui.QComboBox()
        self.ui_type.addItem('MTMR')
        self.ui_type.addItem('MTML')
        self.ui_type.addItem('PSM1')
        self.ui_type.addItem('PSM2')
        self.ui_type.addItem('PSM3')
        # self.ui_type.addItem('ECM')
        ui_hbox_type = QtGui.QHBoxLayout()
        ui_hbox_type.addWidget(ui_type_lable)
        ui_hbox_type.addWidget(self.ui_type)
        self.ui_type.currentIndexChanged.connect(self.slot_type_changed)

        # control mode
        # todo: use a for loop
        gbox = QtGui.QGridLayout()
        ui_btn_idle = QtGui.QPushButton('Idle')
        ui_btn_idle.setCheckable(True)
        ui_btn_idle.setChecked(True)
        gbox.addWidget(ui_btn_idle, 0, 0)
        ui_btn_idle.clicked[bool].connect(self.slot_btn_idle)

        ui_btn_home = QtGui.QPushButton('Home')
        ui_btn_home.setCheckable(True)
        gbox.addWidget(ui_btn_home, 0, 1)
        ui_btn_home.clicked[bool].connect(self.slot_btn_home)

        ui_btn_grav = QtGui.QPushButton('Gravity')
        ui_btn_grav.setCheckable(True)
        gbox.addWidget(ui_btn_grav, 1, 0)
        ui_btn_grav.clicked[bool].connect(self.slot_btn_grav)

        ui_btn_vfix = QtGui.QPushButton('Teleop')
        ui_btn_vfix.setCheckable(True)
        gbox.addWidget(ui_btn_vfix, 1, 1)
        ui_btn_vfix.clicked[bool].connect(self.slot_btn_teleop)

        ui_btn_group = QtGui.QButtonGroup(self._widget)
        ui_btn_group.addButton(ui_btn_idle)
        ui_btn_group.addButton(ui_btn_home)
        ui_btn_group.addButton(ui_btn_grav)
        ui_btn_group.addButton(ui_btn_vfix)
        ui_btn_group.setExclusive(True)

        # connect here
        self.ui_gbox_control = QtGui.QGroupBox('Control MTM')
        self.ui_gbox_control.setLayout(gbox)

        # ---- PSM Control Box ----
        gbox = QtGui.QGridLayout()
        ui_btn_idle = QtGui.QPushButton('Idle')
        ui_btn_idle.setCheckable(True)
        ui_btn_idle.setChecked(True)
        gbox.addWidget(ui_btn_idle, 0, 0)
        ui_btn_idle.clicked[bool].connect(self.slot_btn_idle)

        ui_btn_home = QtGui.QPushButton('Home')
        ui_btn_home.setCheckable(True)
        gbox.addWidget(ui_btn_home, 0, 1)
        ui_btn_home.clicked[bool].connect(self.slot_btn_home)

        ui_btn_grav = QtGui.QPushButton('Teleop')
        ui_btn_grav.setCheckable(True)
        gbox.addWidget(ui_btn_grav, 1, 0)
        ui_btn_grav.clicked[bool].connect(self.slot_btn_teleop)

        ui_btn_vfix = QtGui.QPushButton('Manual')
        ui_btn_vfix.setCheckable(True)
        gbox.addWidget(ui_btn_vfix, 1, 1)
        ui_btn_vfix.clicked[bool].connect(self.slot_btn_manual)

        ui_btn_group = QtGui.QButtonGroup(self._widget)
        ui_btn_group.addButton(ui_btn_idle)
        ui_btn_group.addButton(ui_btn_home)
        ui_btn_group.addButton(ui_btn_grav)
        ui_btn_group.addButton(ui_btn_vfix)
        ui_btn_group.setExclusive(True)

        # connect here
        self.ui_gbox_control_psm = QtGui.QGroupBox('Control PSM')
        self.ui_gbox_control_psm.setLayout(gbox)
        self.ui_gbox_control_psm.setVisible(False)

        # ---- gripper box -----
        ui_hbox_gripper = QtGui.QHBoxLayout()
        ui_gripper_label = QtGui.QLabel('Gripper Angle:')
        ui_hbox_gripper.addWidget(ui_gripper_label)
        ui_gbox_gripper = QtGui.QGroupBox('Gripper')
        ui_gbox_gripper.setLayout(ui_hbox_gripper)

        # ---- joint position group -----
        jnt_pos_hbox = QtGui.QHBoxLayout()
        jp_widgets = []
        jn_widgets = []
        for i in range(7):
            pos_vbox = QtGui.QVBoxLayout()
            ui_ppos = QwtThermo()
            ui_ppos.setScalePosition(QwtThermo.NoScale)
            ui_ppos.setAutoFillBackground(True)
            ui_ppos.setAlarmLevel(0.8)
            ui_ppos.setPipeWidth(20)
            ui_ppos.setValue(0.0)
            ui_ppos.setMinimumSize(0, 40)
            ui_ppos.setRange(0.0, 1.0, False)
            ui_npos = QwtThermo()
            ui_npos.setScalePosition(QwtThermo.NoScale)
            ui_npos.setAlarmLevel(0.8)
            ui_npos.setPipeWidth(20)
            ui_npos.setValue(0.9)
            ui_npos.setMinimumSize(0, 40)
            ui_npos.setRange(1.0, 0.0, False)
            ui_npos.setValue(0.0)
            ui_label_jnt = QtGui.QLabel('J' + str(i))
            pos_vbox.addWidget(ui_ppos)
            pos_vbox.addWidget(ui_npos)
            pos_vbox.addWidget(ui_label_jnt)
            jnt_pos_hbox.addLayout(pos_vbox)
            jp_widgets.append(ui_ppos)
            jn_widgets.append(ui_npos)

        # ui_btn_jnt_pos = QPushButton('J1')
        ui_gbox_jnt_pos = QtGui.QGroupBox('Joint Positions (normalized)')
        ui_gbox_jnt_pos.setLayout(jnt_pos_hbox)
        self.joint_widgets = zip(jp_widgets, jn_widgets)

        # joint torque group
        jnt_eff_hbox = QtGui.QHBoxLayout()
        tp_widgets = []
        tn_widgets = []
        for i in range(7):
            eff_vbox = QtGui.QVBoxLayout()
            ui_peff = QwtThermo()
            ui_peff.setScalePosition(QwtThermo.NoScale)
            ui_peff.setAutoFillBackground(True)
            ui_peff.setAlarmLevel(0.8)
            ui_peff.setPipeWidth(20)
            ui_peff.setValue(0.0)
            ui_peff.setMinimumSize(0, 30)
            ui_peff.setRange(0.0, 1.0, False)
            ui_neff = QwtThermo()
            ui_neff.setScalePosition(QwtThermo.NoScale)
            ui_neff.setAlarmLevel(0.8)
            ui_neff.setPipeWidth(20)
            ui_neff.setValue(0.9)
            ui_neff.setMinimumSize(0, 30)
            ui_neff.setRange(1.0, 0.0, False)
            ui_neff.setValue(0.0)
            ui_label_jnt = QtGui.QLabel('J' + str(i))
            eff_vbox.addWidget(ui_peff)
            eff_vbox.addWidget(ui_neff)
            eff_vbox.addWidget(ui_label_jnt)
            jnt_eff_hbox.addLayout(eff_vbox)
            tp_widgets.append(ui_peff)
            tn_widgets.append(ui_neff)

        ui_gbox_jnt_eff = QtGui.QGroupBox('Joint Torques (normalized)')
        ui_gbox_jnt_eff.setLayout(jnt_eff_hbox)
        self.torque_widgets = zip(tp_widgets, tn_widgets)

        # make widgets colorful
        self.dvrk_green = QColor(87, 186, 142)
        self.dvrk_green_dark = self.dvrk_green.darker()
        self.dvrk_green_light = self.dvrk_green.lighter()
        self.dvrk_blue = QColor(80, 148, 204)
        self.dvrk_blue_dark = self.dvrk_blue.darker()
        self.dvrk_blue_light = self.dvrk_blue.lighter()
        self.dvrk_red = QColor(232, 47, 47)
        self.dvrk_red_dark = self.dvrk_red.darker()
        self.dvrk_red_light = self.dvrk_red.lighter()
        self.dvrk_orange = QColor(255, 103, 43)
        self.dvrk_orange_dark = self.dvrk_orange.darker()

        # joint_bg_color = self.dvrk_blue_dark.darker()
        joint_fill_color = self.dvrk_blue
        joint_alarm_color = self.dvrk_blue_light  # self.dvrk_blue_light
        # torque_bg_color = self.dvrk_green_dark.darker()
        torque_fill_color = self.dvrk_green
        torque_alarm_color = self.dvrk_orange  # self.dvrk_green_light

        for w in jp_widgets + jn_widgets:
            w.setAlarmLevel(0.80)
            w.setFillColor(joint_fill_color)
            w.setAlarmColor(joint_alarm_color)
            p = w.palette()
            # p.setColor(ui_ppos.backgroundRole(), joint_bg_color)
            w.setPalette(p)

        for w in tp_widgets + tn_widgets:
            w.setAlarmLevel(0.66)
            w.setFillColor(torque_fill_color)
            w.setAlarmColor(torque_alarm_color)
            p = w.palette()
            # p.setColor(ui_peff.backgroundRole(), torque_bg_color)
            w.setPalette(p)

        # main layout
        main_layout = QtGui.QVBoxLayout()
        main_layout.addLayout(ui_hbox_type)
        main_layout.addWidget(self.ui_gbox_control)
        main_layout.addWidget(self.ui_gbox_control_psm)
        main_layout.addWidget(ui_gbox_gripper)
        main_layout.addWidget(ui_gbox_jnt_pos)
        main_layout.addWidget(ui_gbox_jnt_eff)
        self._widget.setLayout(main_layout)
        pass

    def init_ros(self):
        # pub
        topic = '/' + self.namespace + '/set_robot_state'
        self.pub_state = rospy.Publisher(topic, String)
        # self.pub_state = rospy.Publisher('/dvrk_psm1/set_robot_state', String)

        # sub
        topic = '/' + self.namespace + '/joint_states'
        self.sub_jnts = rospy.Subscriber(topic, JointState,
                                         self.cb_ros_jnt_states)

        # urdf
        urdf_param = '/' + self.namespace + '/robot_description'
        print urdf_param
        self.urdf = ''
        try:
            self.urdf = rospy.get_param(urdf_param)
        except KeyError:
            rospy.logerr('robot_description not set')
        print self.urdf
        pass

    def shutdown_plugin(self):
        # unregister
        self.pub_state.unregister()
        self.sub_jnts.unregister()

        # del pubs
        del self.pub_state
        del self.sub_jnts
        pass

    def reconfigure_ui(self):
        if 'mtm' in self.namespace:
            self.ui_gbox_control.setVisible(True)
            self.ui_gbox_control_psm.setVisible(False)
        elif 'psm' in self.namespace:
            self.ui_gbox_control.setVisible(False)
            self.ui_gbox_control_psm.setVisible(True)
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO: save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def slot_type_changed(self, index):
        robot_namespaces = ('dvrk_mtmr', 'dvrk_mtml',
                            'dvrk_psm1', 'dvrk_psm2',
                            'dvrk_psm3', 'dvrk_ecm')
        self.namespace = robot_namespaces[index]
        rospy.loginfo('namespace = ' + self.namespace)

        # update ros pub/sub
        self.shutdown_plugin()
        self.init_ros()
        self.reconfigure_ui()
        pass

    def slot_btn_home(self, checked):
        rospy.loginfo('home btn pressed')
        self.pub_state.publish('Home')
        pass

    def slot_btn_idle(self, checked):
        rospy.loginfo('idle btn pressed')
        self.pub_state.publish('Idle')
        pass

    def slot_btn_grav(self, checked):
        rospy.loginfo('grav btn pressed')
        self.pub_state.publish('Gravity')
        pass

    def slot_btn_teleop(self, checked):
        rospy.loginfo('teleop btn pressed')
        self.pub_state.publish('Teleop')
        pass

    def slot_btn_manual(self, checked):
        rospy.loginfo('manual btn pressed')
        self.pub_state.publish('Manual')
        pass

    def cb_ros_jnt_states(self, msg):
        # save to states
        self.jnt_pos = []
        self.jnt_eff = []
        self._jnt_limit_effort = [1, 2, 3, 4, 5, 6, 7]
        self.torque_norm = []

        jnt_limit_min = (-pi/2, -pi/4, 0, -2.27, -pi/2, -1.40, 0, 0)
        jnt_limit_max = (pi/2, pi/4, 0.235, 2.27, pi/2, 1.40, pi/2)

        for i in range(7):
            rng = jnt_limit_max[i] - jnt_limit_min[i]
            pos = 2.0 * (msg.position[i] - jnt_limit_min[i]) / rng - 1.0
            self.jnt_pos.append(pos)

        # print rospy.Time.now()
        pass

    def update_widget_values(self):
        # update btns 
        # print rospy.Time.now()
        for (v, (jp, jn)) in zip(self.jnt_pos, self.joint_widgets):
            jp.setEnabled(True)
            jn.setEnabled(True)
            jp.setValue(v if v >= 0 else 0)
            jn.setValue(-v if v < 0 else 0)

        # update status
        pass

