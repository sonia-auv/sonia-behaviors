#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

from flexbe_core import EventState, Logger

class init_vision_target(EventState):

    '''
        Create the object for the trajectory list

        #> target_list              VisionTarget[]      List of VisionTarget

        <= continue                                     Indicates that the object is created
    '''

    def __init__(self):
        
        super(init_vision_target, self).__init__(outcomes=['continue'],
                                              output_keys=['target_list'])

    def execute(self, userdata):
        Logger.log('Creating an array to fill with vision targets', Logger.REPORT_HINT)
        target_list = []
        userdata.target_list = target_list
        return 'continue'

    def on_exit(self, userdata):
        pass
