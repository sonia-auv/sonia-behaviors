#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool, String

class start_bundle(EventState):

    '''
        Start the record of a bundle with proc_mapping.

        -- target       string      Name of the object that you want to start bundling.
        -- resetBundle  bool        Indicates if you want to clear a current bundle or not.

        <= continue                 Indicates that the recording is started.
    '''

    def __init__(self, target = 'Buoys', resetBundle = True):
        
        super(start_bundle, self).__init__(outcomes=['continue'])
        self.resetBundle = resetBundle
        self.startBundlePub = rospy.Publisher('/proc_mapping/start', String, queue_size= 1)
        self.clearBundlePub = rospy.Publisher('/proc_mapping/clear_bundle', Bool, queue_size= 1)
        self.target = target
    
    def execute(self, userdata):
        Logger.log('Sending start to bundler', Logger.REPORT_HINT)
        if(self.resetBundle):
            self.clearBundlePub.publish(True)
        self.startBundlePub.publish(self.target)
        return 'continue'

    def on_exit(self, userdata):
        pass
        