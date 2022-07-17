#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Int8MultiArray

class init_mission_list(EventState):

    '''
        Initialise the mission array of both submarines. (not ideal be will work)

        -- auv_list             String      Present submarine mission list to do
        -- other_auv_list       String      Other submarine mission list to do   
        
        <=continue                          Both lists have been sent to the proc_underwater_com
    '''

    def __init__(self, auv_list, other_auv_list):
        super(init_mission_list, self).__init__(outcomes=['continue'])

        self.string_auv_list = auv_list
        self.string_other_auv_list = other_auv_list
        self.data_auv_list = Int8MultiArray()
        self.data_other_auv_lsit = Int8MultiArray()

        self.pub = rospy.Publisher("/proc_underwater_com/mission_init", Int8MultiArray, queue_size=10)

    def string_to_int8(self, string):
        data = Int8MultiArray()

        test = string.split(",")
        Logger.log(test, Logger.REPORT_HINT)

    def on_enter(self, userdata):
        pass

    def execute(self, userdata):
        self.data_auv_list = self.string_to_int8(self.string_auv_list)
        self.data_other_auv_lsit = self.string_to_int8(self.string_other_auv_list)
        return 'continue'

    def on_exit(self, userdata):
        self.pub.unregister()