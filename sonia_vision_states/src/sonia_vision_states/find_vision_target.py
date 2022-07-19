#!/usr/bin/env python 
#-*- coding: utf-8 -*-
from time import time
import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import VisionTarget

class find_vision_target(EventState):

    '''
        Verify that the vision target has been found

        -- number_samples       uint8       Number of samples to find the target

        ># filterchain          string      Topic to listen for the target
        ># target               string      Target to find

        <= continue                         Target has been found
    
    '''

    def __init__(self, number_samples):

        super(find_vision_target, self).__init__(outcomes = ['continue'],
                                                 input_keys = ['filterchain', 'target'])
        
        self.param_number_samples = number_samples
        self.start_time = 0

    def vision_cb(self, data):
        if data.header == self.target or data.desc_1 == self.target:
            self.number_of_found += 1

    def on_enter(self, userdata): 
        self.number_of_found = 0
        self.target = userdata.target
        self.get_vision_data = rospy.Subscriber(userdata.filterchain, VisionTarget, self.vision_cb)
        Logger.log('Checking to find the target', Logger.REPORT_HINT)

    def execute(self, userdata):
        if self.number_of_found > self.param_number_samples:
            Logger.log('Target found', Logger.REPORT_HINT)
            return 'continue'

    def on_exit(self, userdata):
        self.get_vision_data.unregister()
