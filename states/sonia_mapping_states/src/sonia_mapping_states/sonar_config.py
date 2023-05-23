#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

import rospy
import dynamic_reconfigure.client

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool

class sonar_config(EventState):

    '''
        State to change the sonar configurations dynamicaly.

        -- gain       uint8     Gain for the sonar acquisition.
        -- range      uint8     Range of the sonar for the acquisition.

        <= continue             Indicates that the sonar is configured.
    '''

    def __init__(self, gain = 100, range = 4):
        super(sonar_config, self).__init__(outcomes=['continue'])
        self.time_launch = time()
        self.gain = gain
        self.range = range

    def on_enter(self, userdata):
        # Dynamic reconfigure client.
        self.client = dynamic_reconfigure.client.Client("provider_sonar", timeout=30, config_callback=self.dyn_callback)
        self.client.update_configuration({ "gain":self.gain, "range":self.range })

    def dyn_callback(self, config):
        Logger.log('Reconfiguring the sonar.', Logger.REPORT_HINT)
        
    def execute(self, userdata):
        time_dif = time() - self.time_launch
        # Time has to experimental values
        if time_dif >= 3.0:
            return 'continue'