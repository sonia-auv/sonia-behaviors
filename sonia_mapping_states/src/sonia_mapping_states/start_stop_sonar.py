#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool

class start_stop_sonar(EventState):

    '''
        State to start or stop the sonar pings.

        -- startStop   bool     Start (true) or stop (false) the pings.

        <= continue             Indicates that the ping are started/stopped.
    '''

    def __init__(self, startStop = True):
        
        super(start_stop_sonar, self).__init__(outcomes=['continue'])
        self.time_launch = time()
        self.startStop = startStop
        self.enablePingPub = rospy.Publisher('/provider_sonar/enable_disable_ping', Bool, queue_size=10)
    
    def on_enter(self, userdata):
        Logger.log('Sending enable ping to sonar', Logger.REPORT_HINT)
        self.enablePingPub.publish(self.startStop)

    def execute(self, userdata):
        time_dif = time() - self.time_launch
        # Time has to experimental values
        if time_dif > 0.5:
            return 'continue'