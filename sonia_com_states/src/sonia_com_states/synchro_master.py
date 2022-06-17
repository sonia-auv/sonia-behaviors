#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
from time import time

from flexbe_core import EventState, Logger
from std_msgs.msg import Int8MultiArray, Float32, Bool
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from sonia_common.msg import AddPose

class synchro_master(EventState):

    '''
        Synchronize a mission with the slave. The master changes depth to reduce the
        chance of hitting each submarine.

        -- mission_id           UInt8       Position of the mission in the array for the mission to sync

        #> depth_change          AddPose    Pose for the new depth requested for the submarine

        <= continue                         Both mission completed.
        <= failed                           Mission failed or already completed by the slave
        <= timeout                          Timeout reached. The other submarine didn't complete the mission
        <= pose_change_depth                The request to change the depth with the output_key "depth_change"
    '''

    def __init__(self, depth_change=True, max_depth_surface=0.5, max_depth_bottom=1.5, min_depth_offset=1,  mission_id=0, timeout=180):
        super(synchro_master, self).__init__(outcomes=['continue', 'failed', 'timeout', 'pose_change_depth'],
                                             output_keys=['depth_change'])
        self.array = Int8MultiArray()
        self.other_array = Int8MultiArray()
        self.depth_change = depth_change
        self.max_depth_surface = max_depth_surface
        self.max_depth_bottom = max_depth_bottom
        self.min_depth_offset = min_depth_offset
        self.mission_id = mission_id - 1
        self.timeout = timeout
        self.actual_z = 0
        self.other_sub_z = 0
        self.request_completed = False
        self.depth_received = False
        self.other_sub_depth_received = False
        self.done_already = False
        self.failed = False
        self.mission_to_do = False
        self.sync_in_progress = False

    def get_depth_cb(self, data):
        self.other_sub_z = data.data
        self.other_sub_depth_received = True

    def get_current_position_cb(self, data):
        self.actual_z = data.pose.pose.position.z
        self.depth_received = True

    def profondeur_change_request(self):
        Logger.log('Other submarine depth is ' + str(self.other_sub_z) + ' and this submarine is ' + \
            str(self.actual_z), Logger.REPORT_HINT)

        # Z Difference used for calculation
        zdiff = self.other_sub_z - self.actual_z

        #Creating the pose for the trajectory
        pose = AddPose()
        pose.position = Point(0, 0, 0)
        pose.orientation = Vector3(0, 0, 0)
        pose.frame = 1
        pose.speed = 0
        pose.fine = 0
        pose.rotation = False

        if abs(zdiff) >= self.min_depth_offset:
            Logger.log('The submarines are at a good depth difference. No need to move to move it.', Logger.REPORT_HINT)
        else:
            Logger.log('The difference is less than ' + str(self.min_depth_offset) + 'm. Move is needed.', Logger.REPORT_HINT)
            
            # The master is over the other submarine
            if zdiff > 0:
                Logger.log('Trying to go up.', Logger.REPORT_HINT)
                if self.actual_z - self.min_depth_offset + zdiff < self.max_depth_surface:
                    Logger.log('The submarine is too close of the surface and could hit the other sub going down.', Logger.REPORT_HINT)
                    self.failed = True
                else:
                    # The goal is to keep the min_depth_offset between the submarine
                    pose.position.z = -self.min_depth_offset + zdiff

            # The master is under the other submarine
            elif zdiff < 0:
                Logger.log('Trying to go down', Logger.REPORT_HINT)
                if self.actual_z + self.min_depth_offset - zdiff > self.max_depth_bottom:
                    Logger.log('The submarine is too close to the bottom and could hit the other sub going up.', Logger.REPORT_HINT)
                    self.failed = True
                else:
                    # The goal is to keep the min_depth_offset between the submarine
                    pose.position.z = self.min_depth_offset - zdiff
            
            # This state should not be able to happend but this has been implemented as a safety measure
            else:
                self.failed = True

        return pose

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
                Logger.log('Verification from the other sub depth', Logger.REPORT_HINT)
                self.get_current_position_sub = rospy.Subscriber('/telemetry/auv_states', Odometry, self.get_current_position_cb)
                self.get_depth_other_sub = rospy.Subscriber('/proc_underwater_com/other_sub_depth', Float32, self.get_depth_cb)
                
                # Demande pour obtenir la profondeur de l'autre sous-marin
                self.depth_request = rospy.ServiceProxy('/proc_underwater_com/depth_request', Empty)
                try:
                    self.depth_request()
                    Logger.log('Request for the other sub depth complete', Logger.REPORT_HINT)
                    self.request_completed = True

                except rospy.ServiceException as exc:
                    Logger.log('Service did not process request: ' + str(exc), Logger.REPORT_HINT)
                    self.failed = True
                
        if msg.data[self.mission_id] > 1 and self.mission_to_do == True:
            Logger.log('Mission has be completed while waiting. Synching the submarines', Logger.REPORT_HINT)
            
            self.sync = rospy.Publisher('/proc_underwater_com/sync_send_msg', Bool, queue_size=2)

            sync_msg = Bool()
            sync_msg.data = True
            self.sync.publish(sync_msg)

            self.sync_in_progress = True
        
    def on_enter(self, userdata):
        self.other_receive_array = rospy.Subscriber('/proc_underwater_com/other_sub_mission_list', Int8MultiArray, self.other_mission_array_cb)
        self.time_start = time()
        
    def execute(self, userdata):
        time_since = time() - self.time_start
        if time_since > self.timeout:
            Logger.log('Timeout Reached', Logger.REPORT_HINT)
            return 'timeout'
        if self.done_already == True or self.failed == True:
            return 'failed'
        if self.sync_in_progress == True:
            return 'continue'
        if self.request_completed == True and self.depth_received == True and self.other_sub_depth_received == True:
            #Stopping to get in the service calling in the callback for the other sub mission list
            self.request_completed = False
            self.get_current_position_sub.unregister()
            self.get_depth_other_sub.unregister()
            
            # Get the pose to change the depth of this submarine
            pose = self.profondeur_change_request()
            
            if self.failed == False and pose.position.z != 0:
                userdata.depth_change = pose
                return 'pose_change_depth'

    def on_exit(self, userdata):
        self.other_receive_array.unregister()