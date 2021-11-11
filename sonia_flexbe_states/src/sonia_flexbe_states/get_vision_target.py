#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from Queue import deque
from flexbe_core import EventState, Logger
from sonia_common.msg import VisionTarget, AddPose
from geometry_msgs.msg import Point, Vector3

class get_vision_target(EventState):

    '''
        Get the movement target from vision filterchain

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

        <= success                              The target has been reached. Ready for action
        <= move                                 Movement ready to do with the pose calculated
        <= failed                               Error in the calculation and loop
    '''

    def __init__(self, bounding_box_pixel, target_width_meter, target_height_meter, ratio_victory, number_of_average, camera, max_mouvement, min_mouvement):
        
        super(get_vision_target, self).__init__(outcomes = ['success', 'move', 'failed'],
                                                input_keys = ['filterchain'],
                                                output_keys = ['pose'])

        self.param_bbp = bounding_box_pixel
        self.param_twm = target_width_meter
        self.param_thm = target_height_meter
        self.param_rv = ratio_victory
        self.param_noa = number_of_average
        self.param_cam = camera
        self.param_mm = max_mouvement
        self.param_min_mouvement = min_mouvement

        self.number_iteration = 0

        # if camera == 2:
        #     self.focal_distance = 0.25
        # else:
        #     self.focal_distance =  0.1651

    def vision_cb(self, vision_data):
        self.vision_x_pixel.append(vision_data.x)
        self.vision_y_pixel.append(vision_data.y)
        self.vision_width_pixel.append(vision_data.width)
        self.vision_height_pixel.append(vision_data.height)

        if  len(self.vision_x_pixel) == self.param_noa and \
            len(self.vision_y_pixel) == self.param_noa and \
            len(self.vision_width_pixel) == self.param_noa and \
            len(self.vision_height_pixel) == self.param_noa:
            self.parse_vision_data()
            Logger.log('Parsing vision data : %d' % self.number_of_sample, Logger.REPORT_HINT)
            self.number_of_sample += 1

    def parse_vision_data(self):
        average_x_pixel = 0.
        average_y_pixel = 0.
        average_width_pixel = 0.
        average_height_pixel = 0.

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
        Logger.log('Alignement for position', Logger.REPORT_HINT)
        # To test to see if working
        new_pose = AddPose()
        surface_image = self.width * self.height
        surface_target = self.param_thm * self.param_thm

        mouvement = surface_target / surface_image

        if mouvement > self.param_mm :
            mouvement = self.param_mm
        
        if self.param_cam == 2 :
            new_pose.position = Point(0.,0.,mouvement)
        else :
            new_pose.position = Point(mouvement,0.,0.)

        return self.fill_pose(new_pose)
    
    def fill_pose(self, pose):
        pose.orientation = Vector3(0.,0.,0.)
        pose.frame = 1
        pose.speed = 5
        pose.fine = 0.
        pose.rotation = True
        return pose

    def on_enter(self, userdata):
        self.vision_x_pixel = deque([], maxlen=self.param_noa)
        self.vision_y_pixel = deque([], maxlen=self.param_noa)
        self.vision_width_pixel = deque([], maxlen=self.param_noa)
        self.vision_height_pixel = deque([], maxlen=self.param_noa)

        self.vision_x_pixel.clear()
        self.vision_y_pixel.clear()
        self.vision_width_pixel.clear()
        self.vision_height_pixel.clear()

        self.position_reached = False
        self.alignement_reached = False
        self.x = 0
        self.y = 0
        self.width = 0
        self.height = 0
        self.number_of_sample = 0

        self.get_vision_data = rospy.Subscriber(userdata.filterchain, VisionTarget, self.vision_cb)

        Logger.log('Starting to gather data', Logger.REPORT_HINT) 

    def execute(self, userdata):
        if self.number_of_sample > self.param_noa:
            self.number_iteration += 1
            Logger.log('Ending', Logger.REPORT_HINT)
            if self.position_reached == True and self.alignement_reached == True:
                return 'success'
            elif self.alignement_reached == False:
                userdata.pose = self.align_with_vision()
                if self.number_iteration < 10 :
                    return 'move'
                else :
                    return 'failed'
            elif self.position_reached == False:
                userdata.pose = self.position_with_vision()
                if self.number_iteration < 10 :
                    return 'move'
                else :
                    return 'failed'
            else:
                return 'failed'

    def on_exit(self, userdata):
        self.get_vision_data.unregister()