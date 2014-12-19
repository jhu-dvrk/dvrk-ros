import os
import rospy

from qt_gui.plugin import Plugin

class dvrkDashboard(Plugin):
    def __init__(self, context):
        super(dvrkDashboard, self).__init__(context)
        print 'init plugin'
        pass
    
