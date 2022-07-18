#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from queue import deque
from flexbe_core import EventState, Logger
from sonia_common.msg import VisionTarget, MissionTimer
from sonia_navigation_states.modules.navigation_utilities import missionTimerFunc

class verify_centroid(EventState):

    '''
        Verify that the vision target is in the bounding box in the parameter

        -- number_of_sample     uint8       Number of the sample to average the centroid
        -- timeout              uint8       Timeout to stop the move

        ># filterchain                      Name of the topic to subcribe for the target
        ># bounding_box                     Size of the bounding box to verify the centroid
        ># header_name                      Name of the header to include in the VisionTarget message

        <= align_complete                   The sub has reached the desired bounding box
        <= timeout_reached                  Timeout has been reached. Didn't find the target
    '''

    def __init__(self, number_sample=10, timeout=30):
        
        super(verify_centroid, self).__init__(outcomes = ['align_complete','timeout_reached'],
                                              input_keys = ['filterchain', 'bounding_box', 'header_name'])
        
        self.param_number_sample = number_sample
        self.param_timeout = timeout
        self.timeout_pub = rospy.Publisher('/sonia_behaviors/timeout', MissionTimer, queue_size=5)
        self.uniqueID = str(time())

        self.vision_x_pixel = deque([], maxlen=self.param_number_sample)
        self.vision_y_pixel = deque([], maxlen=self.param_number_sample)

    def vision_cb(self, vision_data):
        if vision_data.header == self.header_name or vision_data.desc_1 == self.header_name:
            self.vision_x_pixel.append(vision_data.x)
            self.vision_y_pixel.append(vision_data.y)
        if len(self.vision_x_pixel) == self.param_number_sample and \
           len(self.vision_y_pixel) == self.param_number_sample:
            self.verify_centroid()

    def verify_centroid(self):
        average_x_pixel = 0.
        average_y_pixel = 0.
        
        for i in self.vision_x_pixel:
            average_x_pixel += i
        average_x_pixel /= len(self.vision_x_pixel)

        for i in self.vision_y_pixel:
            average_y_pixel += i
        average_y_pixel /= len(self.vision_y_pixel)

        if abs(average_x_pixel) <= self.bounding_box and abs(average_y_pixel) <= self.bounding_box:
            self.at_centroid = True

    def on_enter(self, userdata):
        self.vision_x_pixel.clear()
        self.vision_y_pixel.clear()

        self.x = 0.
        self.y = 0.
        self.at_centroid = False
        self.bounding_box = userdata.bounding_box
        self.header_name = userdata.header_name

        self.get_vision_target = rospy.Subscriber(userdata.filterchain, VisionTarget, self.vision_cb)
        self.start_time = time()
        self.timeout_pub.publish(missionTimerFunc("verify_centroid", self.param_timeout, self.uniqueID, 1))
        Logger.log('Starting the verify the centroid', Logger.REPORT_HINT)
    
    def execute(self, userdata):
        actual = time() - self.start_time
        if self.at_centroid == True:
            self.timeout_pub.publish(missionTimerFunc("verify_centroid", self.param_timeout, self.uniqueID, 2))
            Logger.log('Alignement on centroid complete', Logger.REPORT_HINT)
            return 'align_complete'
        elif actual >= self.param_timeout:
            self.timeout_pub.publish(missionTimerFunc("verify_centroid", self.param_timeout, self.uniqueID, 3))
            Logger.log('Timeout has been reached', Logger.REPORT_HINT)
            return 'timeout_reached'

    def on_exit(self, userdata):
        self.get_vision_target.unregister()