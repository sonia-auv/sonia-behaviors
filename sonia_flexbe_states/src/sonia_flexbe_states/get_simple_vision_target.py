#!/usr/bin/env python 
#-*- coding: utf-8 -*-
from math import sqrt
from time import time
import rospy

from Queue import deque
from flexbe_core import EventState, Logger
from sonia_common.msg import VisionTarget, AddPose
from geometry_msgs.msg import Point, Vector3

class get_simple_vision_target(EventState):

    '''
        Get the movement target from vision filterchain (to moddify)

        -- bounding_box_pixel       uint16      Side of square bounding box for alignement on target in pixel
        -- target_width_meter       float       Width of the target in meters
        -- target_height_meter      float       Height of the target in meters
        -- ratio_victory            float       Ratio of the distance from the target for victory
        -- number_of_average        uint8       Number of image to average before creating a pose
        -- camera                   uint8       1 : front 
                                                2 : bottom
                                                3 : front simulation
                                                4 : bottom simulation
        -- max_mouvement            uint8       Maximum distance allowed to move
        -- min_mouvement            uint8       Minimum distance for a mouvement
        -- return_angle             bool        Return the angle obtained with vision filterchain

        <= success                              The target has been reached. Ready for action
        <= move                                 Movement ready to do with the pose calculated
        <= failed                               Error in the calculation and loop
    '''

    def __init__(self, bounding_box_pixel, image_height=400, image_width=600, ratio_victory=0.5, number_of_average=10, max_mouvement=1, min_mouvement=0.25, timeout=60):
        
        super(get_simple_vision_target, self).__init__(outcomes = ['success', 'align', 'move', 'failed', 'search'],
                                                input_keys = ['filterchain', 'camera_no'],
                                                output_keys = ['pose', 'bounding_box'])

        self.param_bbp = bounding_box_pixel
        self.param_image_height = image_height
        self.param_image_width = image_width
        self.param_rv = ratio_victory
        self.param_noa = number_of_average
        self.param_mm = max_mouvement
        self.param_min_mouvement = min_mouvement
        self.param_timeout = timeout
        
        self.vision_x_pixel = deque([], maxlen=self.param_noa)
        self.vision_y_pixel = deque([], maxlen=self.param_noa)
        self.vision_width_pixel = deque([], maxlen=self.param_noa)
        self.vision_height_pixel = deque([], maxlen=self.param_noa)
        self.vision_angle = deque([], maxlen=self.param_noa)

    def vision_cb(self, vision_data):
        self.vision_x_pixel.append(vision_data.x)
        self.vision_y_pixel.append(vision_data.y)
        self.vision_width_pixel.append(vision_data.width)
        self.vision_height_pixel.append(vision_data.height)
        self.vision_angle.append(vision_data.angle)

        if  len(self.vision_x_pixel) == self.param_noa and \
            len(self.vision_y_pixel) == self.param_noa and \
            len(self.vision_width_pixel) == self.param_noa and \
            len(self.vision_height_pixel) == self.param_noa and \
            len(self.vision_angle) == self.param_noa:
            self.parse_vision_data()
            self.parse_data = True

    def parse_vision_data(self):
        average_x_pixel = 0.
        average_y_pixel = 0.
        average_width_pixel = 0.
        average_height_pixel = 0.
        average_angle = 0.

        for i in self.vision_x_pixel:
            average_x_pixel += i
        average_x_pixel /= len(self.vision_x_pixel)

        for i in self.vision_y_pixel:
            average_y_pixel += i
        average_y_pixel /= len(self.vision_y_pixel)

        for i in self.vision_width_pixel:
            average_width_pixel += i
        average_width_pixel /= len(self.vision_width_pixel)

        for i in self.vision_height_pixel:
            average_height_pixel += i
        average_height_pixel /= len(self.vision_height_pixel)

        for i in self.vision_angle:
            average_angle += i
        average_angle /= len(self.vision_angle)
        self.angle = average_angle

        if average_width_pixel * average_height_pixel > self.param_image_width * self.param_image_height * self.param_rv :
            self.position_reached = True
        else:
            self.position_reached = False
            Logger.log('Width of bounding box (px): %f' %average_width_pixel, Logger.REPORT_HINT)
            Logger.log('Height of bounding box (px): %f' %average_height_pixel, Logger.REPORT_HINT)

        if abs(average_x_pixel) <= self.param_bbp and abs(average_y_pixel) <= self.param_bbp :
            self.alignement_reached = True
        else:
            self.alignement_reached = False
            self.x = average_x_pixel
            self.y = average_y_pixel
            Logger.log('Offset x from reference (px): %f' %self.x, Logger.REPORT_HINT)
            Logger.log('Offset y bounding box (px): %f' %self.y, Logger.REPORT_HINT)

    def align_with_vision(self):
        Logger.log('Alignement on target', Logger.REPORT_HINT)
        #To test to see if working
        new_pose = AddPose()
        mouvement_x = self.x
        mouvement_y = self.y

        if abs(mouvement_x) > self.param_mm :
            if mouvement_x < 0 :
                mouvement_x = -self.param_mm
            else :
                mouvement_x = self.param_mm
        elif abs(mouvement_x) < self.param_min_mouvement :
            if mouvement_x < 0 :
                mouvement_x = -self.param_min_mouvement
            else :
                mouvement_x = self.param_min_mouvement

        if abs(mouvement_y) > self.param_mm :
            if mouvement_y < 0 :
                mouvement_y = -self.param_mm
            else :
                mouvement_y = self.param_mm
        elif abs(mouvement_y) < self.param_min_mouvement :
            if mouvement_y < 0 :
                mouvement_y = -self.param_min_mouvement
            else :
                mouvement_y = self.param_min_mouvement

        if self.param_cam == 2 or self.param_cam == 4 :
            new_pose.position = Point(mouvement_y, mouvement_x, 0.)
        else :
            new_pose.position = Point(0., mouvement_x, -mouvement_y)

        return self.fill_pose(new_pose)
        
    def position_with_vision(self):
        Logger.log('Alignement for position. Creating pose', Logger.REPORT_HINT)
        # To test to see if working
        new_pose = AddPose()        
        if self.param_cam == 2 or self.param_cam == 4 :
            new_pose.position = Point(0.,0.,self.param_mm)
        else :
            new_pose.position = Point(self.param_mm,0.,0.)

        return self.fill_pose(new_pose, 5)
    
    def fill_pose(self, pose, speed):
        pose.orientation = Vector3(0.,0.,0.)
        pose.frame = 1
        pose.speed = speed
        pose.fine = 0.
        pose.rotation = True
        return pose

    def angle_obtained(self):
        pose = AddPose()
        pose.position = Point(0.,0.,0.)
        pose.orientation = Vector3(0.,0.,self.angle)
        pose.frame = 1
        pose.speed = 5
        pose.fine = 0.
        pose.rotation = True
        return pose

    def on_enter(self, userdata):

        self.vision_x_pixel.clear()
        self.vision_y_pixel.clear()
        self.vision_width_pixel.clear()
        self.vision_height_pixel.clear()
        self.vision_angle.clear()

        self.position_reached = False
        self.alignement_reached = False
        self.parse_data = False
        self.x = 0.
        self.y = 0.
        self.angle = 0.
        self.param_cam = userdata.camera_no

        if self.param_cam == 2 or self.param_cam == 4 :
            self.param_ra = True
        else :
            self.param_ra = False

        self.get_vision_data = rospy.Subscriber(userdata.filterchain, VisionTarget, self.vision_cb)
        self.start_time = time()

        Logger.log('Starting to gather data', Logger.REPORT_HINT) 

    def execute(self, userdata):
        actual = time() - self.start_time
        if self.parse_data == True:
            self.parse_data = False
            Logger.log('Ending', Logger.REPORT_HINT)
            if self.position_reached == True and self.alignement_reached == True:
                if self.param_ra == True :
                    userdata.pose = self.angle_obtained()
                return 'success'
            elif self.alignement_reached == False:
                userdata.pose = self.align_with_vision()
                return 'align'
            elif self.position_reached == False:
                userdata.pose = self.position_with_vision()
                return 'move'
            else:
                return 'failed'
        if actual > self.param_timeout :
            return 'search'

    def on_exit(self, userdata):
        userdata.bounding_box = self.param_bbp
        self.get_vision_data.unregister()