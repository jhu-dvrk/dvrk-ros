# Zihan Chen
# 2014-12-19

import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import QtGui
from python_qt_binding import QtCore
from python_qt_binding.Qwt.Qwt import QwtThermo
from std_msgs.msg import String
from sensor_msgs.msg import JointState


# TODO
#  - joint limit database
#  - update topics when type has changed
#  - set robot to bold
#  - subscribe to joint states
#  - get num of jnts from urdf

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
        # ui_type_lable.setFont(QtGui.QFont(QtGui.QFont.Bold))
        ui_type_lable.setAlignment(QtCore.Qt.AlignCenter)
        self.ui_type = QtGui.QComboBox()
        self.ui_type.addItem('MTMR')
        self.ui_type.addItem('MTML')
        self.ui_type.addItem('PSM1')
        self.ui_type.addItem('PSM2')
        self.ui_type.addItem('PSM3')
        self.ui_type.addItem('ECM')
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
        ui_gbox_control = QtGui.QGroupBox('Control')
        ui_gbox_control.setLayout(gbox)

        # joint position group
        jnt_pos_hbox = QtGui.QHBoxLayout()
        for i in range(7):
            pos_vbox = QtGui.QVBoxLayout()
            ui_ppos = QwtThermo()
            ui_ppos.setScalePosition(QwtThermo.NoScale)
            ui_ppos.setAutoFillBackground(True)
            ui_ppos.setAlarmLevel(0.8)
            ui_ppos.setPipeWidth(20)
            ui_ppos.setValue(0.9)
            ui_ppos.setMinimumSize(0, 30)
            ui_npos = QwtThermo()
            ui_npos.setScalePosition(QwtThermo.NoScale)
            ui_npos.setAlarmLevel(0.8)
            ui_npos.setPipeWidth(20)
            ui_npos.setValue(0.9)
            ui_npos.setMinimumSize(0, 30)
            ui_label_jnt = QtGui.QLabel('J' + str(i))
            pos_vbox.addWidget(ui_ppos)
            pos_vbox.addWidget(ui_npos)
            pos_vbox.addWidget(ui_label_jnt)
            jnt_pos_hbox.addLayout(pos_vbox)

        # ui_btn_jnt_pos = QPushButton('J1')
        ui_gbox_jnt_pos = QtGui.QGroupBox('Joint Positions (normalized)')
        ui_gbox_jnt_pos.setLayout(jnt_pos_hbox)

        # joint torque group

        # main layout
        main_layout = QtGui.QVBoxLayout()
        main_layout.addLayout(ui_hbox_type)
        main_layout.addWidget(ui_gbox_control)
        main_layout.addWidget(ui_gbox_jnt_pos)
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
        pass

    def shutdown_plugin(self):
        # unregister
        self.pub_state.unregister()
        self.sub_jnts.unregister()

        # del pubs
        del self.pub_state
        del self.sub_jnts
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

    def cb_ros_jnt_states(self, msg):
        # save to states
        self.jnt_pos = []
        self.jnt_eff = []
        self._jnt_limit_effort = [1, 2, 3, 4, 5, 6, 7]
        self.torque_norm = []

        for i in range(7):
            self.jnt_pos.append(0)
            self.torque_norm

        print rospy.Time.now()
        pass

    def update_widget_values(self):
        # update btns 
        # print rospy.Time.now()
        # update status
        pass

