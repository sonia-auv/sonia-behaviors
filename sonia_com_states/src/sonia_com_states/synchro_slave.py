#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
from time import time

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool

class synchro_slave(EventState):

    '''
        Synchronize a mission with the master. This state needs to be placed before the mission that
        needs to be synchronise.

        -- timeout          uint16      Maximum time to wait for the synch request

        <=continue                      Both mission completed.
        <=timeout                       Timeout reached. The submarine didn't receive the message
    '''

    def __init__(self, timeout=120):
        super(synchro_slave, self).__init__(outcomes=['continue', 'timeout'])
        self.timeout = timeout
        self.message_received = False

    def receive_synchro_msg_cb(self, msg):
        Logger.log('Received synchronisation message from the master', Logger.REPORT_HINT)
        Logger.log('Value of synchro is ' + str(msg.data), Logger.REPORT_HINT)
        
        if msg.data == True:
            self.message_received = True
        
    def on_enter(self, userdata):
        self.other_receive_array = rospy.Subscriber('/proc_underwater_com/sync_requested', Bool, self.receive_synchro_msg_cb)
        self.time_start = time()
        
    def execute(self, userdata):
        time_since = time() - self.time_start
        if time_since > self.timeout:
            Logger.log('Timeout Reached', Logger.REPORT_HINT)
            return 'timeout'
        if self.message_received == True:
            return 'continue' 

    def on_exit(self, userdata):
        self.other_receive_array.unregister()