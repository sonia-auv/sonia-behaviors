#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
from flexbe_core import EventState, Logger

class activate_behavior(EventState):

    '''
        Activate or deactivate behavior
        '''

    def __init__(self, activate=True):
        
        super(activate_behavior, self).__init__(outcomes=['activate', 'desactivate'])
        self.activate = activate

    def on_enter(self, userdata):
        pass

    def execute(self, userdata):
        if self.activate:
            Logger.log('Current behavior activated', Logger.REPORT_HINT)
            return 'activate'
        else:
            Logger.log('Current behavior not activated', Logger.REPORT_HINT)
            return 'desactivate'

    def on_exit(self, userdata):
        pass