#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Int8MultiArray, MultiArrayDimension

class init_mission_list(EventState):

    '''
        Initialise the mission array of both submarines. (not ideal be will work)

        -- auv_list             String      Present submarine mission list to do
        -- other_auv_list       String      Other submarine mission list to do
        -- auv                  String      AUV8 or AUV7
        
        <=continue                          Both lists have been sent to the proc_underwater_com
    '''

    def __init__(self, auv_list, other_auv_list, auv):
        super(init_mission_list, self).__init__(outcomes=['continue'])

        self.string_auv_list = auv_list
        self.string_other_auv_list = other_auv_list
        self.auv = auv

        self.pub = rospy.Publisher("/proc_underwater_com/mission_init", Int8MultiArray, queue_size=10)

    def string_to_int8(self, string):
        data = Int8MultiArray()
        test = string.split(",")
        data.data = list(map(int, test))
        return data

    def on_enter(self, userdata):
        pass

    def execute(self, userdata):
        data_auv_list = self.string_to_int8(self.string_auv_list)
        Logger.log("Submarine mission list : " + str(data_auv_list.data), Logger.REPORT_HINT)
        data_other_auv_list = self.string_to_int8(self.string_other_auv_list)
        Logger.log("Submarine mission list : " + str(data_other_auv_list.data), Logger.REPORT_HINT)

        dim8 = MultiArrayDimension()
        dim7 = MultiArrayDimension()
        dim8.label = "8"
        dim7.label = "7"

        if self.auv == "AUV8":
            data_auv_list.layout.dim.append(dim8)
            data_other_auv_list.layout.dim.append(dim7)
        elif self.auv == "AUV7":
            data_auv_list.layout.dim.append(dim7)
            data_other_auv_list.layout.dim.append(dim8)
        else:
            data_auv_list.layout.dim.append(dim8)
            data_other_auv_list.layout.dim.append(dim7)

        self.pub.publish(data_auv_list)
        self.pub.publish(data_other_auv_list)

        return 'continue'

    def on_exit(self, userdata):
        self.pub.unregister()