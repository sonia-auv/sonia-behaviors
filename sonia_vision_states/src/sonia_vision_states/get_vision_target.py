#!/usr/bin/env python 
#-*- coding: utf-8 -*-
from cmath import pi
from math import sqrt
from time import time

from psutil import pid_exists
import rospy
import numpy

from queue import deque
from flexbe_core import EventState, Logger
import sonia_navigation_states.modules.navigation_utilities as navUtils
from sonia_common.msg import VisionTarget, MultiAddPose, AddPose, MissionTimer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3

class get_vision_target(EventState):

    '''
        Get the movement target from vision filterchain

        -- center_bounding_box_pixel_height     uint16          Height of center bounding box for alignement on target in pixel
        -- center_bounding_box_pixel_width      uint16          Width of center bounding box for alignement on target in pixel
        -- deep_bounding_box_pixel_height       uint16          Height of bounding box for alignement on target in pixel
        -- deep_bounding_box_pixel_width        uint16          Width of bounding box for alignement on target in pixel
        -- sift_bounding_box_pixel_height       uint16          Height of bounding box for alignement on target in pixel
        -- sift_bounding_box_pixel_width        uint16          Width of bounding box for alignement on target in pixel
        -- simple_bounding_box_pixel_height     uint16          Height of bounding box for alignement on target in pixel
        -- simple_bounding_box_pixel_width      uint16          Width of bounding box for alignement on target in pixel
        -- image_height                         float           Width of the image in pixel
        -- image_width                          float           Height of the image in pixel
        -- deep_number_of_average               uint8           Number of deep image to average before creating a pose
        -- sift_number_of_average               uint8           Number of sift image to average before creating a pose
        -- simple_number_of_average             uint8           Number of simple image to average before creating a pose
        -- max_mouvement                        uint8           Maximum distance allowed to move
        -- min_mouvement                        float           Minimum distance for a mouvement
        -- long_rotation                        bool            False : Quick path for rotation
                                                                True : Long path for rotation
        -- timeout                              uint8           Time to stop looking at this position
        -- speed_profile                        uint8           Speed profile to move
                                                                0 : normal
                                                                1 : slow
                                                                2 : fast

        ># deep_filterchain                     string          Topic to listen to get the deep target
        ># sift_filterchain                     string          Topic to listen to get the sift target
        ># simple_filterchain                   string          Topic to listen to get the simple target
        ># camera_no                            uint8           1 : front 
                                                                2 : bottom
                                                                3 : front simulation
                                                                4 : bottom simulation       
        ># deep_target                          string          Target to align to
        ># sift_target                          string          Target to align to
        ># simple_target                        string          Target to align to
        ># input_trajectory                     MultiAddPose    Input trajectory

        #> output_trajectory                    MultiAddPose    Output trajectory
        #> camera                               uint8           0 : None
                                                                1 : bottom AUV8
                                                                2 : bottom AUV7

        <= success                                              The target has been reached. Ready for action
        <= move                                                 Movement ready to do with the pose calculated
        <= failed                                               Error in the calculation and loop
        <= search                                               No target found, looking for it with research algo
    '''     

    def __init__(self, center_bounding_box_pixel_height, center_bounding_box_pixel_width, deep_bounding_box_pixel_height,
                 deep_bounding_box_pixel_width, sift_bounding_box_pixel_height, sift_bounding_box_pixel_width, simple_bounding_box_pixel_height,
                 simple_bounding_box_pixel_width, image_height=400, image_width=600, deep_number_of_average=10, sift_number_of_average=10, 
                 simple_number_of_average=10, max_mouvement=1, min_mouvement=0.1, long_rotation=False, timeout=10, speed_profile=0):
        
        super(get_vision_target, self).__init__(outcomes = ['success', 'align', 'move', 'failed', 'search'],
                                                input_keys = ['deep_filterchain', 'sift_filterchain', 'simple_filterchain', 'camera_no', 'target'],
                                                output_keys = ['camera'])

        self.param_center_bbp_height = center_bounding_box_pixel_height
        self.param_center_bbp_width = center_bounding_box_pixel_width

        self.param_deep_bbp_height = deep_bounding_box_pixel_height
        self.param_deep_bbp_width = deep_bounding_box_pixel_width
        
        self.param_sift_bbp_height = sift_bounding_box_pixel_height
        self.param_sift_bbp_width = sift_bounding_box_pixel_width
        
        self.param_simple_bbp_height = simple_bounding_box_pixel_height
        self.param_simple_bbp_width = simple_bounding_box_pixel_width
        
        self.param_deep_bbp_area = self.param_deep_bbp_height * self.param_deep_bbp_width
        self.param_sift_bbp_area = self.param_sift_bbp_height * self.param_sift_bbp_width
        self.param_simple_bbp_area = self.param_simple_bbp_height * self.param_simple_bbp_width
        
        self.param_image_height = image_height
        self.param_image_width = image_width
        
        self.param_deep_number_of_average = deep_number_of_average
        self.param_sift_number_of_average = sift_number_of_average
        self.param_simple_number_of_average = simple_number_of_average
        
        self.param_max_mouvement = max_mouvement
        self.param_min_mouvement = min_mouvement
        
        self.param_long_rotation = long_rotation
        self.param_timeout = timeout
        self.param_speed_profile = speed_profile

        self.timeout_pub = rospy.Publisher('/sonia_behaviors/timeout', MissionTimer, queue_size=5)

        self.nombre_enter = 0
        
        self.deep_vision = deque([], maxlen=self.param_deep_number_of_average)
        self.sift_vision = deque([], maxlen=self.param_deep_number_of_average)
        self.simple_vision = deque([], maxlen=self.param_deep_number_of_average)

    def on_enter(self, userdata):

        self.vision_x_pixel.clear()
        self.vision_y_pixel.clear()
        self.vision_width_pixel.clear()
        self.vision_height_pixel.clear()
        self.vision_angle.clear()

        self.position_reached = False
        self.alignement_reached = False

        self.deep_parse_data = False
        self.sift_parse_data = False
        self.simple_parse_data = False

        self.deep_target = VisionTarget()
        self.sift_target = VisionTarget()
        self.simple_target = VisionTarget()

        self.target = userdata.target
        self.position_z = 0.3

        if userdata.camera_no == 2 or userdata.camera_no == 4 :
            self.cam_bottom = True
        else :
            self.cam_bottom = False

        self.get_deep_data = rospy.Subscriber(userdata.deep_filterchain, VisionTarget, self.deep_vision_cb)
        self.get_sift_data = rospy.Subscriber(userdata.sift_filterchain, VisionTarget, self.sift_vision_cb)
        self.get_simple_data = rospy.Subscriber(userdata.simple_filterchain, VisionTarget, self.simple_vision_cb)
        self.get_position = rospy.Subscriber('/proc_nav/auv_states', Odometry, self.position_cb)
        self.start_time = time()
        self.timeout_pub.publish(navUtils.missionTimerFunc("get_vision_target", self.param_timeout, self.start_time, 1))

        self.nombre_enter += 1
        Logger.log('Starting attempt ' + str(self.nombre_enter), Logger.REPORT_HINT) 

    def position_cb(self, data):
        self.position_z = data.pose.pose.position.z

    def append_vision(self, vision_method, vision_data):
        pixel_data = navUtils.vision_pixel()
        if vision_data.header == self.target or vision_data.desc_1 == self.target:
            pixel_data.x = vision_data.x
            pixel_data.y = vision_data.y
            pixel_data.width = vision_data.width
            pixel_data.height = vision_data.height
            pixel_data.angle = vision_data.angle
            
            vision_method.append(pixel_data)

    def deep_vision_cb(self, vision_data):
        self.append_vision(self.deep_vision, vision_data)

        if  len(self.deep_vision.x) == self.param_deep_number_of_average:
            self.get_deep_data.unregister()
            self.deep_target = self.parse_vision_data(vision_data.header, vision_data.desc_1, vision_data.desc_2)
            self.deep_parse_data = True
    
    def sift_vision_cb(self, vision_data):
        self.append_vision(self.sift_vision, vision_data)

        if  len(self.sift_vision.x) == self.param_sift_number_of_average:
            self.get_sift_data.unregister()
            self.sift_target = self.parse_vision_data(vision_data.header, vision_data.desc_1, vision_data.desc_2)
            self.sift_parse_data = True

    def simple_vision_cb(self, vision_data):
        self.append_vision(self.simple_vision, vision_data)

        if  len(self.simple_vision.x) == self.param_simple_number_of_average:
            self.get_simple_data.unregister()
            self.simple_target = self.parse_vision_data(vision_data.header, vision_data.desc_1, vision_data.desc_2)
            self.simple_parse_data = True

    def parse_vision_data(self, header, desc_1, desc_2):
        temp_target = VisionTarget()
        temp_target.x = sum(self.vision_x_pixel)/len(self.vision_x_pixel)
        temp_target.y = sum(self.vision_y_pixel)/len(self.vision_y_pixel)
        temp_target.width = sum(self.vision_width_pixel)/len(self.vision_width_pixel)
        temp_target.height = sum(self.vision_height_pixel)/len(self.vision_height_pixel)
        temp_target.angle = sum(self.vision_angle)/len(self.vision_angle)
        temp_target.header = header
        temp_target.desc_1 = desc_1
        temp_target.desc_2 = desc_2

        return temp_target

    def swap(target):
        swap = target.x
        target.x = target.y
        target.y = swap

    def compare(self):

        self.deep_average_area_pixel = self.deep_target.width*self.deep_target.height
        self.sift_average_area_pixel = self.sift_target.width*self.sift_target.height
        self.simple_average_area_pixel = self.simple_target.width*self.simple_target.height

        Logger.log('Area of deep target (px): %f' %self.deep_average_area_pixel, Logger.REPORT_HINT)
        Logger.log('Area of deep bounding box (px): %f' %self.param_deep_bbp_area, Logger.REPORT_HINT)
        Logger.log('Area of sift target (px): %f' %self.sift_average_area_pixel, Logger.REPORT_HINT)
        Logger.log('Area of sift bounding box (px): %f' %self.param_sift_bbp_area, Logger.REPORT_HINT)
        Logger.log('Area of simple target (px): %f' %self.simple_average_area_pixel, Logger.REPORT_HINT)
        Logger.log('Area of simple bounding box (px): %f' %self.simple_average_area_pixel, Logger.REPORT_HINT)

        # if self.average_area_pixel > self.param_bbp_area :
        #     self.position_reached = True
        #     Logger.log('Position reached', Logger.REPORT_HINT)
        # else:
        #     self.position_reached = False

        if self.cam_bottom:
            self.swap(self.deep_target)
            self.swap(self.sift_target)
            self.swap(self.simple_target)

        # Logger.log('x average: %f' %abs(self.average_x_pixel), Logger.REPORT_HINT)
        # Logger.log(self.param_center_bbp_width/2, Logger.REPORT_HINT)
        # Logger.log('y average: %f' %abs(self.average_y_pixel), Logger.REPORT_HINT)
        # Logger.log(self.param_center_bbp_height/2, Logger.REPORT_HINT)

        # if abs(self.average_y_pixel) <= self.param_center_bbp_height/2 and abs(self.average_x_pixel) <= self.param_center_bbp_width/2 :
        #     self.alignement_reached = True
        #     Logger.log('Alignement reached', Logger.REPORT_HINT)
        # else:
        #     self.alignement_reached = False
        
        # self.parse_data = True

    def execute(self, userdata):
        actual = time() - self.start_time
        if self.simple_parse_data == True:
            self.parse_data = False
            if self.position_reached == True and self.alignement_reached == True:
                if self.cam_bottom == True :
                    userdata.angle = self.average_angle
                    userdata.output_trajectory = userdata.input_trajectory
                    userdata.camera = 1
                self.timeout_pub.publish(navUtils.missionTimerFunc("get_simple_vision_target", self.param_timeout, self.start_time, 2))
                return 'success'
            elif self.alignement_reached == False:
                if self.cam_bottom:
                    userdata.output_trajectory = self.align_bottom_with_vision()
                else:
                    userdata.output_trajectory = self.align_front_with_vision()
                self.timeout_pub.publish(navUtils.missionTimerFunc("get_simple_vision_target", self.param_timeout, self.start_time, 2))
                return 'align'
            elif self.position_reached == False:
                userdata.output_trajectory = self.position_with_vision()
                self.timeout_pub.publish(navUtils.missionTimerFunc("get_simple_vision_target", self.param_timeout, self.start_time, 2))
                return 'move'
            else:
                self.timeout_pub.publish(navUtils.missionTimerFunc("get_simple_vision_target", self.param_timeout, self.start_time, 4))
                return 'failed'
        if actual > self.param_timeout :
            self.timeout_pub.publish(navUtils.missionTimerFunc("get_simple_vision_target", self.param_timeout, self.start_time, 3))
            return 'search'

    def on_exit(self, userdata):
        pass
