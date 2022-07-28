#!/usr/bin/env python 
#-*- coding: utf-8 -*-
from math import sqrt
from time import time
import rospy
import numpy

from queue import deque
from flexbe_core import EventState, Logger
import sonia_navigation_states.modules.navigation_utilities as navUtils
from sonia_common.msg import VisionTarget, MissionTimer

from geometry_msgs.msg import Point, Vector3

class get_vision_target_list(EventState):

    '''
        Get the movement target from vision topic

        -- number_of_average                    uint8           Number of image to average before creating a pose
        -- timeout                              uint8           Time to stop looking at this position

        ># topic                                string          Topic to listen to get the target
        ># camera_no                            uint8           1 : front 
                                                                2 : bottom
                                                                3 : front simulation
                                                                4 : bottom simulation       
        ># target                               string          Target to align to
        ># target_list_in                       VisionTarget[]  List of targets
        
        ># target_list_out                      VisionTarget[]  List of targets                     
        #> camera                               uint8           0 : None
                                                                1 : bottom AUV8
                                                                2 : bottom AUV7

        <= success                                              The target has been reached. Ready for action
        <= lost_target                                          No result from this topic
    '''     

    def __init__(self, number_of_average=10, timeout=10):
        
        super(get_vision_target_list, self).__init__(outcomes = ['success', 'lost_target'],
                                                input_keys = ['topic', 'camera_no', 'target', 'target_list_in'],
                                                output_keys = ['camera','target_list_out'])


        self.param_number_of_average = number_of_average
        self.param_timeout = timeout

        self.timeout_pub = rospy.Publisher('/sonia_behaviors/timeout', MissionTimer, queue_size=5)

        self.nombre_enter = 0
        
        self.vision_x_pixel = deque([], maxlen=self.param_number_of_average)
        self.vision_y_pixel = deque([], maxlen=self.param_number_of_average)
        self.vision_width_pixel = deque([], maxlen=self.param_number_of_average)
        self.vision_height_pixel = deque([], maxlen=self.param_number_of_average)
        self.vision_angle = deque([], maxlen=self.param_number_of_average)

    def on_enter(self, userdata):
        self.vision_x_pixel.clear()
        self.vision_y_pixel.clear()
        self.vision_width_pixel.clear()
        self.vision_height_pixel.clear()
        self.vision_angle.clear()

        self.position_reached = False
        self.alignement_reached = False
        self.parse_data = False
        self.vision_target = VisionTarget()
        self.target = userdata.target
        self.target_list_out = userdata.target_list_in

        self.position_z = 0.3

        if userdata.camera_no == 2 or userdata.camera_no == 4 :
            self.cam_bottom = True
        else :
            self.cam_bottom = False

        self.get_vision_data = rospy.Subscriber(userdata.topic, VisionTarget, self.vision_cb)
        self.start_time = time()
        self.timeout_pub.publish(navUtils.missionTimerFunc("get_vision_target_list", self.param_timeout, self.start_time, 1))

        self.nombre_enter += 1
        Logger.log('Starting attempt ' + str(self.nombre_enter), Logger.REPORT_HINT) 

    def vision_cb(self, vision_data):
        if vision_data.header == self.target or vision_data.desc_1 == self.target or vision_data.desc_2 == self.target:
            self.vision_x_pixel.append(vision_data.x)
            self.vision_y_pixel.append(vision_data.y)
            self.vision_width_pixel.append(vision_data.width)
            self.vision_height_pixel.append(vision_data.height)
            self.vision_angle.append(vision_data.angle)

        if  len(self.vision_x_pixel) == self.param_number_of_average:
            self.get_vision_data.unregister()
            self.parse_vision_data()

    def parse_vision_data(self):
        self.vision_target.x = sum(self.vision_x_pixel)/len(self.vision_x_pixel)
        self.vision_target.y = sum(self.vision_y_pixel)/len(self.vision_y_pixel)
        self.vision_target.width = sum(self.vision_width_pixel)/len(self.vision_width_pixel)
        self.vision_target.height = sum(self.vision_height_pixel)/len(self.vision_height_pixel)
        self.vision_target.angle = sum(self.vision_angle)/len(self.vision_angle)

        self.vision_target_area = self.vision_target.width * self.vision_target.height
        
        if self.cam_bottom:
            swap = self.vision_target.x
            self.vision_target.x = self.vision_target.y
            self.vision_target.y = swap

        self.parse_data = True

    def execute(self, userdata):
        actual = time() - self.start_time

        if self.parse_data == True:
            self.parse_data = False
            self.target_list_out.append(self.vision_target)
            userdata.target_list_out = self.target_list_out
            if self.cam_bottom == True :
                userdata.angle = self.average_angle
                userdata.camera = 1
            self.timeout_pub.publish(navUtils.missionTimerFunc("get_vision_target_list", self.param_timeout, self.start_time, 2))
            return 'success'

        if actual > self.param_timeout :
            self.timeout_pub.publish(navUtils.missionTimerFunc("get_vision_target_list", self.param_timeout, self.start_time, 3))
            return 'lost_target'

    def on_exit(self, userdata):
        pass
