#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
from time import time

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool
from sonia_common.msg import MissionTimer
from sonia_navigation_states.src.sonia_navigation_states.modules.navigation_utilities import missionTimerFunc

class synchro_receive(EventState):

    '''
        Synchronize a mission with the master. This state needs to be placed before the mission that
        needs to be synchronise.

        -- timeout          uint16      Maximum time to wait for the synch request

        <=continue                      Message received
        <=timeout                       Timeout reached. The submarine didn't receive the message
    '''

    def __init__(self, timeout=120):
        super(synchro_receive, self).__init__(outcomes=['continue', 'timeout'])
        self.timeout = timeout
        self.message_received = False
        self.timeout_pub = rospy.Publisher('/sonia_behaviors/timeout', MissionTimer, queue_size=5)
        self.uniqueID = str(time())

    def receive_synchro_msg_cb(self, msg):
        Logger.log('Received synchronisation message from the master', Logger.REPORT_HINT)
        Logger.log('Value of synchro is ' + str(msg.data), Logger.REPORT_HINT)
        
        if msg.data == True:
            self.message_received = True
        
    def on_enter(self, userdata):
        self.other_receive_array = rospy.Subscriber('/proc_underwater_com/sync_requested', Bool, self.receive_synchro_msg_cb)
        self.time_start = time()
        self.timeout_pub.publish(missionTimerFunc("synchro_receive", self.param_timeout, self.uniqueID, 1))
        
    def execute(self, userdata):
        time_since = time() - self.time_start
        if time_since > self.timeout:
            self.timeout_pub.publish(missionTimerFunc("synchro_receive", self.param_timeout, self.uniqueID, 3))
            Logger.log('Timeout Reached', Logger.REPORT_HINT)
            return 'timeout'
        if self.message_received == True:
            self.timeout_pub.publish(missionTimerFunc("synchro_receive", self.param_timeout, self.uniqueID, 2))
            return 'continue' 

    def on_exit(self, userdata):
        self.other_receive_array.unregister()