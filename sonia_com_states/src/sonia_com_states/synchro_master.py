#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
from time import time

from flexbe_core import EventState, Logger
from std_msgs.msg import Int8MultiArray

class synchro_master(EventState):

    '''
        Synchronize a mission with the slave. The master changes depth to reduce the
        chance of hitting each submarine.

        -- mission_id           uint8   Position of the mission in the array for the previous mission

        <=continue                      Both mission completed.
        <=failed                        Mission failed or already completed by the slave
        <=timeout                       Timeout reached. The other submarine didn't complete the mission
    '''

    def __init__(self, mission_id=0, timeout=300):
        super(synchro_master, self).__init__(outcomes=['continue', 'failed', 'timeout'])
        self.other_array = Int8MultiArray()
        self.mission_id = mission_id
        self.timeout = timeout
        self.message_received_other = False

    def other_mission_array_cb(self, msg):
        Logger.log('Received the updated state of the other sub', Logger.REPORT_HINT)
        Logger.log(str(msg.data), Logger.REPORT_HINT)
        
        if msg.data[self.mission_id] > 1 and self.mission_to_do == False:
            Logger.log('Mission already completed. You too slow!', Logger.REPORT_HINT)
            self.done_already = True
        if msg.data[self.mission_id] < 1:
            Logger.log('Mission has been failed. The other submarine is at an unknow position.', Logger.REPORT_HINT)
            self.failed = True
        if msg.data[self.mission_id] == 1 and self.mission_to_do == False:
            Logger.log('Mission is planned and not completed yet.', Logger.REPORT_HINT)
            self.mission_to_do = True
            #Ajouter le changement de profondeur pour le sub (À voir avec Alec)
        if msg.data[self.mission_id] > 1 and self.mission_to_do == True:
            Logger.log('Mission has be completed while waiting.', Logger.REPORT_HINT)
            #Envoi d'un message de synchronisation
            self.sync_in_progress = True
        
    def on_enter(self, userdata):
        self.other_receive_array = rospy.Subscriber('/proc_underwater_com/other_sub_mission_list', Int8MultiArray, self.other_mission_array_cb)
        self.time_start = time()
        self.done_already = False
        self.failed = False
        self.mission_to_do = False
        self.sync_in_progress = False
        
    def execute(self, userdata):
        time_since = time() - self.time_start
        if time_since > self.timeout:
            Logger.log('Timeout Reached', Logger.REPORT_HINT)
            return 'timeout'
        if self.done_already == True or self.failed == True:
            return 'failed'
        if self.sync_in_progress == True:
            return 'continue'     

    def on_exit(self, userdata):
        self.other_receive_array.unregister()