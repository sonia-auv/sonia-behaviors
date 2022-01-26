#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import ActivateAllPS
from geometry_msgs.msg import Pose, Point, Quaternion

class toggling_motors(EventState):

    '''
        Set the intial position of the sub for simulation only

        <= continue                         Indicates completion of the calculation.

        '''

    def __init__(self, simulation=False, toggle=False):
        
        super(toggling_motors, self).__init__(outcomes=['continue'])

        self.toggling_motors_pub = rospy.Publisher('/provider_power/activate_all_ps', ActivateAllPS, queue_size=2)

        self.param_simulation = simulation
        self.param_toggle = toggle

    def execute(self, userdata):
        if self.param_simulation == True:
            Logger.log('Toggling motors activation', Logger.REPORT_HINT)
            self.toggling_motors_pub.publish(ActivateAllPS(49,0,self.param_toggle))
        else:
            Logger.log('Not in simulation. No need for toggling motors activation', Logger.REPORT_HINT)
        return 'continue'

    def on_exit(self, userdata):
        pass
