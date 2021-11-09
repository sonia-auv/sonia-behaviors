#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from statistics import mean
import rospy

from Queue import deque
from flexbe_core import EventState, Logger
from sonia_common.msg import VisionTarget
from geometry_msgs.msg import Point, Vector3

class get_vision_target(EventState):

    '''

    '''

    def __init__(   self, bouding_box_pixel, target_width_meter, target_height_meter,\
                    ratio_victory, number_of_average, camera, max_mouvement):
        
        super(get_vision_target, self).__init__(outcomes = ['success', 'move', 'failed'],
                                                input_keys = ['filterchain'],
                                                output_keys = ['pose'])
        
        self.param_bbp = bouding_box_pixel
        self.param_twm = target_width_meter
        self.param_thm = target_height_meter
        self.param_rv = ratio_victory
        self.param_noa = number_of_average
        self.param_cam = camera
        self.param_mm = max_mouvement

        # if camera == 2:
        #     self.focal_distance = 0.25
        # else:
        #     self.focal_distance =  0.1651

        if  len(self.vision_x_pixel) == self.param_noa and \
            len(self.vision_y_pixel) == self.param_noa and \
            len(self.average_width_pixel) == self.param_noa and \
            len(self.vision_height_pixel) == self.param_noa:
            self.parse_vision_data()
            self.number_of_sample += 1

    def parse_vision_data(self):
        average_x_pixel = mean(self.vision_x_pixel)
        average_y_pixel = mean(self.vision_y_pixel)
        average_width_pixel = mean(self.vision_width_pixel)
        average_height_pixel = mean(self.vision_height_pixel)

        pixel_to_meter = self.param_twm / average_width_pixel

        if average_width_pixel * average_height_pixel > self.param_twm * self.param_thm * self.param_rv :
            self.position_reached = True
        else:
            self.position_reached = False
            self.width = average_width_pixel * pixel_to_meter
            self.height = average_height_pixel * pixel_to_meter

        if abs(average_x_pixel) <= self.param_bbp and abs(average_y_pixel) <= self.param_bbp :
            self.alignement_reached = True
        else:
            self.alignement_reached = False
            self.x = average_x_pixel * pixel_to_meter
            self.y = average_y_pixel * pixel_to_meter

    def align_with_vision(self):
        
        #To test to see if working
        mouvement_x = self.x
        mouvement_y = self.y

        if mouvement_x > 1 :
            mouvement_x = 1
        if mouvement_y > 1 :
            mouvement_y = 1
        if self.param_cam == 2 :
            self.new_pose = []
        
    def position_with_vision(self):

        # To test to see if working
        surface_image = self.width * self.height
        surface_target = self.param_thm * self.param_thm

        mouvement = surface_target / surface_image

        if mouvement > 1 :
            mouvement = 1
        
        if self.param_cam == 2 :
            self.new_pose = [0,0,mouvement,0,0,0,1]
        else :
            self.new_pose = [mouvement, 0,0,0,0,0,1]

    def on_enter(self, userdata):
        self.vision_x_pixel = deque([], maxlen=self.param_noa)
        self.vision_y_pixel = deque([], maxlen=self.param_noa)
        self.vision_width_pixel = deque([], maxlen=self.param_noa)
        self.alignement_reached = False
        self.x = 0
        self.y = 0
        self.width = 0
        self.height = 0
        self.number_of_sample = 0

        self.new_pose = []
        
        self.param_filterchain = userdata.filterchain
        self.get_vision_data = rospy.Subscriber(self.param_filter_chain, VisionTarget, self.vison_cb)   

    def execute(self, userdata):
        Logger.log('Starting to gather data', Logger.REPORT_HINT)
        if self.number_of_sample > self.param_noa:
            Logger.log('Ending', Logger.REPORT_HINT)
            if self.position_reached == True and self.alignement_reached == True:
                return 'success'
            elif self.alignement_reached == False:
                self.align_with_vision()
                userdata.pose = self.new_pose
                return 'move'
            elif self.position_reached == False:
                self.position_with_vision()
                userdata.pose = self.new_pose
                return 'move'
            else:
                return 'failed'

    def on_exit(self, userdata):
        self.get_vision_data.unregister()