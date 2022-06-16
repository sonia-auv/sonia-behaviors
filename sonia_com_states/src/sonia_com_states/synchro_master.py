#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
from time import time

from flexbe_core import EventState, Logger
from std_msgs.msg import Int8MultiArray
from nav_msgs.msg import Odometry
from sonia_common.msg import AddPose

class synchro_master(EventState):

    '''
        Synchronize a mission with the slave. The master changes depth to reduce the
        chance of hitting each submarine.

        -- mission_id           UInt8       Position of the mission in the array for the previous mission

        #> depth_change          AddPose    Pose for the new depth requested for the submarine

        <= continue                         Both mission completed.
        <= failed                           Mission failed or already completed by the slave
        <= timeout                          Timeout reached. The other submarine didn't complete the mission
    '''

    def __init__(self, depth_change=True,  mission_id=0, timeout=300):
        super(synchro_master, self).__init__(outcomes=['continue', 'failed', 'timeout'])
        self.other_array = Int8MultiArray()
        self.depth_change = depth_change
        self.mission_id = mission_id
        self.timeout = timeout
        self.actual_z = 0
        self.message_received_other = False

    def get_current_position_cb(self, data):
        self.actual_z = data.pose.pose.position.z

    def profondeur_verification(self):
        # self.depth_request = rospy.ServiceProxy()
        # try:
        #     self.depth_request()

        # except rospy.ServiceException as exc:
        #     Logger.log('Service did not process request: ' + str(exc), Logger.REPORT_HINT)
        #     self.failed = True

        Logger.log('Other submarine depth is ' + str() + ' and this submarine is ')

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

            # Vérification de la profondeur de l'autre sous-marin
            if self.depth_change == True:
            self.get_current_position_sub = rospy.Subscriber('/telemetry/auv_states', Odometry, self.get_current_position_cb)
            
            



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