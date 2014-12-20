import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding.QtGui import QWidget, QPushButton, QVBoxLayout

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
        self._widget = QWidget()
        self._widget.setObjectName('dvrkDashboardUI')

        btnHome = QPushButton('Home', self._widget)
        vbox = QVBoxLayout()
        vbox.addWidget(btnHome)
        self._widget.setLayout(vbox)

        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
            
        # add widget to the user interface
        context.add_widget(self._widget)

        pass
    
