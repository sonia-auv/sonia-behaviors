#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
import math
from time import time

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils
from flexbe_core import EventState, Logger
from sonia_common.msg import MultiAddPose, PingAngles

class get_angle_from_hydro(EventState):

    def __init__(self, frequency = 25000, timeout = 10, frequency_timeout = 10):
        super(get_angle_from_hydro, self).__init__(outcomes=['continue', 'failed'],
                                                     output_keys=['angle'])
        self.hydro_angle = -1
        self.hydro_freq = -1
        self.launch_time = 0
        self.time_diff = 0
        self.frequency_timeout = frequency_timeout
        self.timeout = timeout
        self.frequency = int(frequency)

    def get_hydro_data_cb(self, data):
        self.hydro_angle = data.heading
        self.hydro_freq = data.frequency
        self.is_alive = True

    def on_enter(self, userdata):
        self.is_alive = False
        self.frequency_match = False
        self.get_hydro_data_sub = rospy.Subscriber('/proc_hydrophone/ping', PingAngles, self.get_hydro_data_cb)
        self.launch_time = time()

        try:       
            while self.time_diff < self.frequency_timeout :
                rospy.wait_for_message('/proc_hydrophone/ping', PingAngles, timeout=self.timeout)
                if self.hydro_freq == self.frequency:
                    Logger.log('Frequency wanted detected by hydro : ' + str(self.hydro_freq), Logger.REPORT_HINT)
                    Logger.log('Angle detected by hydro : ' + str(self.hydro_angle), Logger.REPORT_HINT)
                    self.frequency_match = True
                    break
                self.time_diff = time() - self.launch_time

        except:
            Logger.log('No information given by hydro', Logger.REPORT_HINT)
            pass
        pass

    def execute(self, userdata):
        if self.is_alive and self.frequency_match:
            userdata.angle = self.hydro_angle
            return 'continue'
        elif not self.frequency_match and self.is_alive:
            Logger.log('Frequency not wanted detected by hydro : ' + str(self.hydro_freq), Logger.REPORT_HINT)
            return 'failed'
        else :
            return 'failed'

    def on_exit(self, userdata):
        self.get_hydro_data_sub.unregister()
        pass