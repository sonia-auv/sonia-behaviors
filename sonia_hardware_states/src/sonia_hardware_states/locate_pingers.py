#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
from math import atan, cos, sin, radians, degrees
from time import time

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils
from flexbe_core import EventState, Logger
from sonia_common.msg import MultiAddPose, PingAngles

class locate_pingers(EventState):

    def __init__(self, move_dist = 1):
        super(locate_pingers, self).__init__(outcomes=['continue', 'failed'],
                                                     input_keys=['angle1', 'angle2'],
                                                     output_keys=['real_angle', 'distance'])        
        self.real_angle = 0
        self.distance = 0
        self.distance_hydro_center = 0.16
        self.move_dist = move_dist

    def on_enter(self, userdata):
        self.angle1 = radians(float(userdata.angle1))
        self.angle2 = radians(float(userdata.angle2 + 6))
        
    def execute(self, userdata):
        if self.angle1 != self.angle2:
            try:
                self.distance = self.move_dist * sin(self.angle1) / (self.angle2 - self.angle1)
                self.real_angle = degrees(atan(sin(self.angle2) / (self.distance_hydro_center * sin(self.angle2 - self.angle1) / (self.move_dist * sin(self.angle1)) + cos(self.angle2))))          
                userdata.distance = self.distance
                userdata.real_angle = self.real_angle
                Logger.log('The angle to the pinger (IMU) is ' + str(self.real_angle), Logger.REPORT_HINT)
                Logger.log('The distance to the pinger is ' + str(self.distance), Logger.REPORT_HINT)
                return 'continue'
            except:
                Logger.log('Math problem', Logger.REPORT_HINT)
                return 'failed'
        else:
            Logger.log('Both angles are equals', Logger.REPORT_HINT)
            return 'failed'
        

    def on_exit(self, userdata):
        pass