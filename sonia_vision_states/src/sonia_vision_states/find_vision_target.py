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
        -- timeout              uint8       Time to stop looking at this position

        ># filterchain          string      Topic to listen for the target

        <= continue                         Target has been found
        <= failed                           Continue to search for the target
    
    '''

    def __init__(self, number_samples, timeout):

        super(find_vision_target, self).__init__(outcomes = ['continue', 'failed'],
                                                 input_keys = ['filterchain'])
        
        self.param_number_samples = number_samples
        self.param_timeout = timeout

    def vision_cb(self, data):
        self.number_of_found += 1

    def on_enter(self, userdata): 
        self.number_of_found = 0
        self.get_vision_data = rospy.Subscriber(userdata.filterchain, VisionTarget, self.vision_cb)
        self.start_time = time()
        Logger.log('Checking to find the target for %d seconds' %self.param_timeout, Logger.REPORT_HINT)

    def execute(self, userdata):
        actual = time()-self.start_time    
        if self.number_of_found > self.param_number_samples:
            Logger.log('Target found', Logger.REPORT_HINT)
            return 'continue'
        if actual > self.param_timeout:
            Logger.log('No target found', Logger.REPORT_HINT)
            return 'failed'

    def on_exit(self, userdata):
        self.get_vision_data.unregister()