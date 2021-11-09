#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from statistics import mean
import rospy

from Queue import deque
from flexbe_core import EventState, Logger
from sonia_common.msg import VisionTarget

class get_vision_target(EventState):

    '''

    '''

    def __init__(self, bouding_box_width_pixel, bouding_box_height_pixel, target_width_meter, target_height_meter, ratio_victory, number_of_average):
        
        super(get_vision_target, self).__init__(outcomes = ['success', 'move', 'failed'],
                                                input_keys = ['filterchain'],
                                                output_keys = ['pose'])
        
        self.param_bbwp = bouding_box_width_pixel
        self.param_bbhp = bouding_box_height_pixel
        self.param_twm = target_width_meter
        self.param_thm = target_height_meter
        self.param_rv = ratio_victory
        self.param_noa = number_of_average

        self.position_reached = False
        self.alignement_reached = False
        self.number_of_sample = 0

    def vision_cb(self, vision_data):
        self.vision_x_pixel.append(vision_data.x)
        self.vision_y_pixel.append(vision_data.y)
        self.vision_width_pixel.append(vision_data.width)
        self.vision_height_pixel.append(vision_data.height)

        if  len(self.vision_x_pixel) == self.param_noa and \
            len(self.vision_y_pixel) == self.param_noa and \
            len(self.average_width_pixel) == self.param_noa and \
            len(self.vision_height_pixel) == self.param_noa:
            self.parse_vision_data()
            self.number_of_sample += 1

    def parse_vision_data(self):
        self.average_x_pixel = mean(self.vision_x_pixel)
        self.average_y_pixel = mean(self.vision_y_pixel)
        self.average_width_pixel = mean(self.vision_width_pixel)
        self.average_height_pixel = mean(self.vision_height_pixel)

    def align_with_vision(self):
        self.new_pose = 0

        #TO DO add script to align to the centroid

    def position_with_vision(self):
        self.new_pose = 0

        #TO DO add script to get closer to the target

    def on_enter(self, userdata):
        self.vision_x_pixel = deque([], maxlen=self.param_noa)
        self.vision_y_pixel = deque([], maxlen=self.param_noa)
        self.vision_width_pixel = deque([], maxlen=self.param_noa)
        self.vision_height_pixel = deque([], maxlen=self.param_noa)
        
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
