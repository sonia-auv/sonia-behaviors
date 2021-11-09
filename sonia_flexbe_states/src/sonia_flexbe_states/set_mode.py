#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import sleep, time
import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import UInt8

class set_mode(EventState):

    '''
        State to change the control mode.
        [...]
        
        -- mode             uint8       choose the control mode

        ># positionX        uint8       Input to the calculation function.

        #> output_value     object      The result of the calculation.

        <= done                         Indicates completion of the calculation.

        '''

    def __init__(self, mode):
        
        super(set_mode, self).__init__(outcomes=['continue', 'failed'])

        self.param_mode = mode
        self.set_mode_pub = rospy.Publisher('/proc_control/set_mode', UInt8, queue_size=2)

    def on_enter(self, userdata):

        self.set_mode_pub.publish(self.param_mode)
        Logger.log('setting mode %s' %self.param_mode,Logger.REPORT_HINT)

    def execute(self, userdata):
        Logger.log('waiting 3 sec',Logger.REPORT_HINT)
        sleep(3)
        return 'continue'
        # TO DO : add handshake

    def on_exit(self, userdata):
        pass
