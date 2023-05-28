#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
from time import sleep

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool

class synchro_send(EventState):

    '''
        State used by the master submarine to send a sync request to
        the other submarine.
        Competition 2022 -> Master = AUV8 & Slave = AUV7
    
    <= continue                     Synchronisation message sent
    '''

    def __init__(self):
        super(synchro_send, self).__init__(outcomes=['continue'])
        self.sync = rospy.Publisher('/proc_underwater_com/send_sync_request', Bool, queue_size=2)
    
    def on_enter(self, userdata):
        pass
    
    def execute(self, userdata):
        Logger.log("Sending synchroniszation message to the other submarine", Logger.REPORT_HINT)

        sync_msg = Bool()
        sync_msg.data = True
        self.sync.publish(sync_msg)

        # This sleep is needed to ensure that the message is published
        sleep(1)
        return 'continue'

    def on_exit(self, userdata):
        pass