#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool

class start_bundle(EventState):

    '''
        Create the object for the trajectory

        <= continue             Indicates that the trajectory will be compute
        <= failed               Indicates that the waypoints aren't correctly
    '''

    def __init__(self, resetBundle = True):
        
        super(start_sonar, self).__init__(outcomes=['continue'])
        self.time_launch = time()
        self.resetBundle = resetBundle
        self.startBundlePub = rospy.Publisher('/proc_mapping/start_stop', Bool, queue_size=10)
        self.clearBundlePub = rospy.Publisher('/proc_mapping/clear_bundle', Bool, queue_size=10)
    
    def on_enter(self, userdata):
        Logger.log('Sending start to bundler', Logger.REPORT_HINT)
        if(self.resetBundle):
            self.clearBundlePub.publish(True)
        self.startBundlePub.publish(True)

    def execute(self, userdata):
        time_dif = time() - self.time_launch
        # Time has to experimental values
        if time_dif > 0.5:
            return 'continue'