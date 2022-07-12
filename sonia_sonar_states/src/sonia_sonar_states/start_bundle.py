#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool, String

class start_bundle(EventState):

    '''
        Create the object for the trajectory

        <= continue             Indicates that the trajectory will be compute
        <= failed               Indicates that the waypoints aren't correctly
    '''

    def __init__(self,Obstacle2Search = 'Buoys', resetBundle = True):
        
        super(start_bundle, self).__init__(outcomes=['continue'])
        self.time_launch = time()
        self.resetBundle = resetBundle
        self.startBundlePub = rospy.Publisher('/proc_mapping/start', String, queue_size= 1)
        self.clearBundlePub = rospy.Publisher('/proc_mapping/clear_bundle', Bool, queue_size= 1)
        self.Obstacle2Search = Obstacle2Search
    
    def execute(self, userdata):
        Logger.log('Sending start to bundler', Logger.REPORT_HINT)
        if(self.resetBundle):
            self.clearBundlePub.publish(True)
        self.startBundlePub.publish(self.Obstacle2Search)
        return 'continue'

    def on_exit(self, userdata):
        pass
        