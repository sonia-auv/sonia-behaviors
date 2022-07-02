#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Int8MultiArray

class get_update(EventState):

    '''
        Get the state for the mission for the other submarine

        <=continue                  Array has been retrieve by the underwater_com

        #> mission_array            State array (Int8MultiArray)
    '''

    def __init__(self):
        super(get_update, self).__init__(outcomes=['continue'],
                                         output_keys=['mission_array'])

        self.array = Int8MultiArray()
        self.message_received = False

    def mission_array_cb(self, msg):
        Logger.log('Received the updated state', Logger.REPORT_HINT)
        Logger.log(str(msg.data), Logger.REPORT_HINT)
        self.array = msg.data
        self.message_received = True

    def on_enter(self, userdata):
        self.receive_array = rospy.Subscriber('/proc_underwater_com/other_sub_mission_list', Int8MultiArray, self.mission_array_cb)

    def execute(self, userdata):
        if self.message_received == True:
            userdata.mission_array = self.array
            return 'continue'

    def on_exit(self, userdata):
        self.receive_array.unregister()