#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool

class stop_move(EventState):

    '''
        Stop an ongoing mouvement (hand break)
    '''

    def __init__(self, timeout=3):

        super(stop_move, self).__init__(outcomes=['continue', 'failed'])

        self.param_timeout = timeout
        self.reset_trajectory = rospy.Publisher('/proc_control/reset_trajectory', Bool, queue_size=2)

    def target_reached_cb(self, data):
        self.target_reached = data.data

    def on_enter(self, userdata):
        self.target_reached = False
        self.start_time = time()
        self.target_reached_sub = rospy.Subscriber('/proc_control/target_reached', Bool, self.target_reached_cb)
        
        Logger.log('Stopping mouvement', Logger.REPORT_HINT)
        self.reset_trajectory.publish(Bool(True))

    def execute(self, userdata):
        actual_time = time()-self.start_time
        if actual_time > self.param_timeout:
            if self.target_reached == True:
                Logger.log('Mouvement has been stopped properly', Logger.REPORT_HINT)
                return 'continue'
            else:
                Logger.log('Submarine hasnt reached target after stopping', Logger.REPORT_HINT)
                return 'failed'

    def on_exit(self, userdata):
        self.target_reached_sub.unregister()