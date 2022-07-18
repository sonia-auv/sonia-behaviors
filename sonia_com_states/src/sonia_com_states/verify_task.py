#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
from time import time

from flexbe_core import EventState, Logger
from std_msgs.msg import Int8MultiArray

class verify_task(EventState):

    '''
        Verify if the mission in parameter needs to be completed or skipped. Will wait for little
        while and will continue with the task.

        -- mission          uint8   Position in the mission array to check if the task needs to be done

        <= to_do                    Task is assigned to this submarine.
        <= skip                     Task is not assigned skip it.
    '''

    def __init__(self, mission, timeout=3):
        super(verify_task, self).__init__(outcomes=['to_do', 'skip'])

        self.mission = mission
        self.timeout = timeout
        self.message_received = False

    def mission_array_cb(self, msg):
        Logger.log('Received the task for this submarine', Logger.REPORT_HINT)
        Logger.log(str(msg.data), Logger.REPORT_HINT)
        self.array = msg.data
        self.message_received = True

    def on_enter(self, userdata):
        self.receive_array = rospy.Subscriber('/proc_underwater_com/sub_mission_list', Int8MultiArray, self.mission_array_cb)
        self.time_start = time()

    def execute(self, userdata):
        diff = time() - self.time_start
        if diff <= self.timeout:
            if self.message_received == True:
                if self.array[self.mission] == 1:
                    return 'to_do'
                else:
                    return 'skip'
        else:
            return 'to_do'

    def on_exit(self, userdata):
        self.receive_array.unregister()