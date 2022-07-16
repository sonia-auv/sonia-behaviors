#!/usr/bin/env python 
#-*- coding: utf-8 -*-
from math import sqrt
from time import time
import rospy

from queue import deque
from flexbe_core import EventState, Logger
import sonia_navigation_states.src.sonia_navigation_states.modules.navigation_utilities as navUtils
from sonia_common.msg import VisionTarget, MultiAddPose, AddPose, MissionTimer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3

class get_simple_vision_target(EventState):

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
        ># target                       string          Target to align to
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
        
        super(get_simple_vision_target, self).__init__(outcomes = ['success', 'align', 'move', 'failed', 'search'],
                                                input_keys = ['filterchain', 'camera_no', 'target', 'input_trajectory'],
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

        self.timeout_pub = rospy.Publisher('/sonia_behaviors/timeout', MissionTimer, queue_size=5)
        self.uniqueID = str(time())

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
        # self.x = 0.
        # self.y = 0.
        self.average_x_pixel = 0.
        self.average_y_pixel = 0.
        self.average_width_pixel = 0.
        self.average_height_pixel = 0.
        self.angle = 0.
        self.target = userdata.target

        self.position_z = 0.3

        if userdata.camera_no == 2 or userdata.camera_no == 4 :
            self.cam_bottom = True
        else :
            self.cam_bottom = False

        self.get_vision_data = rospy.Subscriber(userdata.filterchain, VisionTarget, self.vision_cb)
        self.get_position = rospy.Subscriber('/proc_nav/auv_states', Odometry, self.position_cb)
        self.start_time = time()
        self.timeout_pub.publish(navUtils.missionTimerFunc("get_simple_vision_target", self.param_timeout, self.uniqueID, 1))

        self.nombre_enter += 1
        Logger.log('Starting attempt ' + str(self.nombre_enter), Logger.REPORT_HINT) 

    def position_cb(self, data):
        self.position_z = data.pose.pose.position.z

    def vision_cb(self, vision_data):
        if vision_data.header == self.target or vision_data.desc_1 == self.target:
            self.vision_x_pixel.append(vision_data.x)
            self.vision_y_pixel.append(vision_data.y)
            self.vision_width_pixel.append(vision_data.width)
            self.vision_height_pixel.append(vision_data.height)
            self.vision_angle.append(vision_data.angle)

        if  len(self.vision_x_pixel) == self.param_number_of_average:
            self.get_vision_data.unregister()
            self.parse_vision_data()

    def parse_vision_data(self):
        self.average_x_pixel = sum(self.vision_x_pixel)/len(self.vision_x_pixel)
        self.average_y_pixel = sum(self.vision_y_pixel)/len(self.vision_y_pixel)
        self.average_width_pixel = sum(self.vision_width_pixel)/len(self.vision_width_pixel)
        self.average_height_pixel = sum(self.vision_height_pixel)/len(self.vision_height_pixel)
        self.angle = sum(self.vision_angle)/len(self.vision_angle)

        if self.cam_bottom:
            swap = self.average_x_pixel
            self.average_x_pixel = self.average_y_pixel
            self.average_y_pixel = swap
        
        Logger.log('Area of target (px): %f' %(self.average_height_pixel*self.average_width_pixel), Logger.REPORT_HINT)
        Logger.log('Area of bounding box (px): %f' %(self.param_bbp_height*self.param_bbp_width), Logger.REPORT_HINT)

        if self.average_width_pixel * self.average_height_pixel > self.param_bbp_height * self.param_bbp_width :
            self.position_reached = True
            Logger.log('Position reached', Logger.REPORT_HINT)
        else:
            self.position_reached = False
            #Logger.log('Width of target (px): %f' %average_width_pixel, Logger.REPORT_HINT)
            #Logger.log('Height of target (px): %f' %average_height_pixel, Logger.REPORT_HINT)

        Logger.log('x average: %f' %abs(self.average_x_pixel), Logger.REPORT_HINT)
        Logger.log(self.param_bbp_width/2, Logger.REPORT_HINT)
        Logger.log('y average: %f' %abs(self.average_y_pixel), Logger.REPORT_HINT)
        Logger.log(self.param_bbp_height/2, Logger.REPORT_HINT)

        if abs(self.average_y_pixel) <= self.param_bbp_height/2 and abs(self.average_x_pixel) <= self.param_bbp_width/2 :
            self.alignement_reached = True
            Logger.log('Alignement reached', Logger.REPORT_HINT)
        else:
            self.alignement_reached = False
            # self.x = average_x_pixel
            # self.y = average_y_pixel
        
        self.parse_data = True

    def align_with_vision(self):
        Logger.log('Alignement on target. Creating pose', Logger.REPORT_HINT)

        new_traj = MultiAddPose()
        new_pose = AddPose()
        mouvement_x = self.average_x_pixel / (self.param_image_width*self.position_z)
        mouvement_y = self.average_y_pixel / (self.param_image_height*self.position_z)

        Logger.log('Déplacement x : %f' %mouvement_x, Logger.REPORT_HINT)
        Logger.log('Déplacement y : %f' %mouvement_y, Logger.REPORT_HINT)

        if self.cam_bottom :
            new_pose.position = Point(mouvement_x, mouvement_y, 0.)
        else :
            new_pose.position = Point(0., mouvement_x, -mouvement_y)

        new_traj.pose.append(self.fill_pose(new_pose))
        return (new_traj)
        
    def position_with_vision(self):
        Logger.log('Getting closer. Creating pose', Logger.REPORT_HINT)

        new_traj = MultiAddPose()
        new_pose = AddPose()
        if self.cam_bottom:
            new_pose.position = Point(0.,0.,self.param_max_mouvement/5)
        else :
            mouvement_x = self.param_max_mouvement*(1-((self.average_width_pixel*self.average_height_pixel)/(self.param_bbp_height*self.param_bbp_width)))
            Logger.log('Déplacement x : %f' %mouvement_x, Logger.REPORT_HINT)
            new_pose.position = Point(mouvement_x,0.,0.)

        new_traj.pose.append(self.fill_pose(new_pose))
        return (new_traj)

    def fill_pose(self, pose):
        return navUtils.addpose(pose.position.x, pose.position.y, pose.position.z, 0., 0., 0., 1, self.param_speed_profile, 0, self.param_long_rotation)

    def execute(self, userdata):
        actual = time() - self.start_time
        if self.parse_data == True:
            self.parse_data = False
            #Logger.log('Checking for position and alignement', Logger.REPORT_HINT)
            if self.position_reached == True and self.alignement_reached == True:
                if self.cam_bottom == True :
                    userdata.angle = self.angle
                    userdata.output_trajectory = userdata.input_trajectory
                    userdata.camera = 1
                self.timeout_pub.publish(navUtils.missionTimerFunc("get_simple_vision_target", self.param_timeout, self.uniqueID, 2))
                return 'success'
            elif self.alignement_reached == False:
                userdata.output_trajectory = self.align_with_vision()
                return 'align'
            elif self.position_reached == False:
                userdata.output_trajectory = self.position_with_vision()
                return 'move'
            else:
                self.timeout_pub.publish(navUtils.missionTimerFunc("get_simple_vision_target", self.param_timeout, self.uniqueID, 4))
                return 'failed'
        if actual > self.param_timeout :
            self.timeout_pub.publish(navUtils.missionTimerFunc("get_simple_vision_target", self.param_timeout, self.uniqueID, 3))
            return 'search'

    def on_exit(self, userdata):
        pass