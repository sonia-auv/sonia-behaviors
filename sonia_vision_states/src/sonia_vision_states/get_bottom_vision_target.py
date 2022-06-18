#!/usr/bin/env python 
#-*- coding: utf-8 -*-
from math import sqrt
from time import time
import rospy

from queue import deque
from flexbe_core import EventState, Logger
import sonia_navigation_states.modules.navigation_utilities as navUtils
from sonia_common.msg import VisionTarget, MultiAddPose, AddPose
from geometry_msgs.msg import Point, Vector3

class get_bottom_vision_target(EventState):

    '''
        Get the movement target from vision filterchain

        -- bounding_box_pixel_height    uint16          height of bounding box for alignement on target in pixel
        -- bounding_box_pixel_width     uint16          Width of square bounding box for alignement on target in pixel
        -- target_width_meter           float           Width of the target in meters
        -- target_height_meter          float           Height of the target in meters
        -- number_of_average            uint8           Number of image to average before creating a pose
        -- max_mouvement                uint8           Maximum distance allowed to move
        -- min_mouvement                float           Minimum distance for a mouvement
        -- long_rotation                bool            False : Quick path for rotation
                                                        True : Long path for rotation
        -- timeout                      uint8           Time to stop looking at this position
        -- speed_profile                uint8           Speed profile to move
                                                        0 : normal
                                                        1 : slow
                                                        2 : fast

        ># filterchain                  string          Topic to listen to get the target
        ># camera_no                    uint8           1 : front 
                                                        2 : bottom
                                                        3 : front simulation
                                                        4 : bottom simulation       
        ># header_name                  string          Header name to filter result
        ># input_trajectory             MultiAddPose    Input trajectory

        #> output_trajectory            MultiAddPose    Output trajectory
        #> camera                       uint8           0 : None
                                                        1 : bottom AUV8
                                                        2 : bottom AUV7
        #> angle                        float           Angle to rotate

        <= success                                      The target has been reached. Ready for action
        <= move                                         Movement ready to do with the pose calculated
        <= failed                                       Error in the calculation and loop
    '''

    def __init__(self, bounding_box_pixel_height, bounding_box_pixel_width, image_height=400, image_width=600, number_of_average=10, max_mouvement=1, min_mouvement=0.1, long_rotation=False, timeout=10, speed_profile=0):
        
        super(get_bottom_vision_target, self).__init__(outcomes = ['success', 'align', 'move', 'failed', 'search'],
                                                input_keys = ['filterchain', 'camera_no', 'header_name', 'input_trajectory'],
                                                output_keys = ['output_trajectory', 'camera', 'angle'])

        self.param_bbp_height = bounding_box_pixel_height
        self.param_bbp_width = bounding_box_pixel_width
        self.param_image_height = image_height
        self.param_image_width = image_width
        self.param_number_of_average = number_of_average
        self.param_max_mouvement = max_mouvement
        self.param_min_mouvement = min_mouvement
        self.param_long_rotation = long_rotation
        self.param_timeout = timeout
        self.param_speed_profile = speed_profile

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
        self.x = 0.
        self.y = 0.
        self.angle = 0.
        self.header_name = userdata.header_name

        if userdata.camera_no == 2 or userdata.camera_no == 4 :
            self.cam_bottom = True
        else :
            self.cam_bottom = False

        self.get_vision_data = rospy.Subscriber(userdata.filterchain, VisionTarget, self.vision_cb)
        self.start_time = time()

        self.nombre_enter += 1
        Logger.log('Starting attempt ' + str(self.nombre_enter), Logger.REPORT_HINT) 

    def vision_cb(self, vision_data):
        if vision_data.header == self.header_name or vision_data.desc_1 == self.header_name:
            self.vision_x_pixel.append(vision_data.x)
            self.vision_y_pixel.append(vision_data.y)
            self.vision_width_pixel.append(vision_data.width)
            self.vision_height_pixel.append(vision_data.height)
            self.vision_angle.append(vision_data.angle)

        if  len(self.vision_x_pixel) == self.param_number_of_average:
            self.get_vision_data.unregister()
            self.parse_vision_data()
            self.parse_data = True

    def parse_vision_data(self):
        average_x_pixel = sum(self.vision_x_pixel)/len(self.vision_x_pixel)
        average_y_pixel = sum(self.vision_y_pixel)/len(self.vision_y_pixel)
        average_width_pixel = sum(self.vision_width_pixel)/len(self.vision_width_pixel)
        average_height_pixel = sum(self.vision_height_pixel)/len(self.vision_height_pixel)
        self.angle = sum(self.vision_angle)/len(self.vision_angle)

        if self.cam_bottom:
            swap = average_x_pixel
            average_x_pixel = average_y_pixel
            average_y_pixel = swap

        if average_width_pixel * average_height_pixel > self.param_bbp_height * self.param_bbp_width :
            self.position_reached = True
            Logger.log('Position reached', Logger.REPORT_HINT)
        else:
            self.position_reached = False
            Logger.log('Width of target (px): %f' %average_width_pixel, Logger.REPORT_HINT)
            Logger.log('Height of target (px): %f' %average_height_pixel, Logger.REPORT_HINT)
            Logger.log('Area of target (px): %f' %(average_height_pixel*average_width_pixel), Logger.REPORT_HINT)
            Logger.log('Area of bounding box (px): %f' %(self.param_bbp_height*self.param_bbp_width), Logger.REPORT_HINT)

        if abs(average_y_pixel) <= self.param_bbp_height and abs(average_x_pixel) <= self.param_bbp_width :
            self.alignement_reached = True
            Logger.log('Alignement reached', Logger.REPORT_HINT)
        else:
            self.alignement_reached = False
            self.x = average_x_pixel
            self.y = average_y_pixel

    def align_with_vision(self):
        Logger.log('Alignement on target. Creating pose', Logger.REPORT_HINT)

        new_traj = MultiAddPose()
        new_pose = AddPose()
        mouvement_x = self.x / self.param_image_width
        mouvement_y = self.y / self.param_image_height

        Logger.log('Déplacement x : %f' %mouvement_x, Logger.REPORT_HINT)
        Logger.log('Déplacement y : %f' %mouvement_y, Logger.REPORT_HINT)

        if self.cam_bottom :
            new_pose.position = Point(mouvement_x, mouvement_y, 0.)
        else :
            new_pose.position = Point(0., mouvement_x, -mouvement_y/4)

        new_traj.pose.append(self.fill_pose(new_pose))
        return (new_traj)
        
    def position_with_vision(self):
        Logger.log('Getting closer. Creating pose', Logger.REPORT_HINT)

        new_traj = MultiAddPose()
        new_pose = AddPose()
        if self.cam_bottom:
            new_pose.position = Point(0.,0.,self.param_max_mouvement/5)
        else :
            new_pose.position = Point(self.param_min_mouvement,0.,0.)

        new_traj.pose.append(self.fill_pose(new_pose))
        return (new_traj)

    def fill_pose(self, pose):
        return navUtils.addpose(pose.position.x, pose.position.y, pose.position.z, 0., 0., 0., 1, self.param_speed_profile, 0, self.param_long_rotation)

    def execute(self, userdata):
        actual = time() - self.start_time
        if self.parse_data:
            self.parse_data = False
            Logger.log('Checking for position and alignement', Logger.REPORT_HINT)
            if self.position_reached and self.alignement_reached:
                if self.cam_bottom:
                    userdata.angle = self.angle
                    userdata.output_trajectory = userdata.input_trajectory
                    userdata.camera = 1
                return 'success'
            elif not self.alignement_reached:
                userdata.output_trajectory = self.align_with_vision()
                return 'align'
            elif not self.position_reached:
                userdata.output_trajectory = self.position_with_vision()
                return 'move'
            else:
                return 'failed'
        if actual > self.param_timeout :
            return 'search'

    def on_exit(self, userdata):
        pass