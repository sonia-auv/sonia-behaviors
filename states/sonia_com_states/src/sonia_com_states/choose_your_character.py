#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
from time import sleep

from flexbe_core import EventState, Logger

class choose_your_character(EventState):

    '''
        Define the outcomes that will be selected for the submarines. For 2022, this is for the first tasks
        on AUV7 and AUV8. With the string

        -- submarine           string   "AUV7" or "AUV8"

        <=auv8                          Selected AUV8
        <=auv7                          Selected AUV7
    '''

    def __init__(self, submarine='AUV8'):
        super(choose_your_character, self).__init__(outcomes=['auv8', 'auv7'])
        self.submarine = submarine

    def on_enter(self, userdata):
        pass

    def execute(self, userdata):
        Logger.log("The submarine selected for the next tasks is " + str(self.submarine), Logger.REPORT_HINT)

        if self.submarine == 'AUV7':
            return 'auv7'
        elif self.submarine == 'AUV8':
            return 'auv8'
        else:
            Logger.log("The submarine choseen doesn't exist. You stupid. AUV8 is taken by default", Logger.REPORT_HINT)
            return 'auv8'
    
    def on_exit(self, userdata):
        pass
