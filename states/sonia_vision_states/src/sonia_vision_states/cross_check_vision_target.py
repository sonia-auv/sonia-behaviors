#!/usr/bin/env python 
#-*- coding: utf-8 -*-
from time import time
import rospy
import numpy

from flexbe_core import EventState, Logger
from sonia_common.msg import VisionTarget, MissionTimer, MultiAddPose, AddPose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import sonia_navigation_states.modules.navigation_utilities as navUtils

class cross_check_vision_target(EventState):

    '''
        Verify that the vision target has been found

        -- center_bounding_box_pixel_height     uint16          Height of center bounding box for alignement on target in pixel
        -- center_bounding_box_pixel_width      uint16          Width of center bounding box for alignement on target in pixel
        -- bounding_box_pixel_height            uint16          Height of bounding box for alignement on target in pixel
        -- bounding_box_pixel_width             uint16          Width of bounding box for alignement on target in pixel
        -- image_height                         float           Width of the image in pixel
        -- image_width                          float           Height of the image in pixel
        -- max_mouvement                        uint8           Maximum distance allowed to move
        -- min_mouvement                        float           Minimum distance for a mouvement
        -- long_rotation                        bool            False : Quick path for rotation
                                                                True : Long path for rotation
        -- speed_profile                        uint8           Speed profile to move
                                                                0 : normal
                                                                1 : slow
                                                                2 : fast
        -- timeout                              uint8           Time to stop looking at this position

        ># target_list_in           VisionTarget    List of VisionTarget
        ># input_trajectory         MultiAddPose    Input tajectory

        ># target_list_out          VisionTarget    List of VisionTarget
        #> output_trajectory        MultiAddPose    Output tajectory

        <= success                                  Succeeded to align
        <= align                                    Move to align
        <= lost_target                              Target lost
    
    '''

    def __init__(self, center_bounding_box_pixel_height, center_bounding_box_pixel_width, bounding_box_pixel_height,
                 bounding_box_pixel_width, image_height=400, image_width=600, max_mouvement=1, 
                 min_mouvement=0.1, long_rotation=False, speed_profile=0, timeout=10):

        super(cross_check_vision_target, self).__init__(outcomes = ['success', 'align', 'lost_target'],
                                                        input_keys = ['target_list_in', 'input_trajectory', 'camera_no'],
                                                        output_keys = ['target_list_out', 'output_trajectory'])

        self.param_center_bbp_height = center_bounding_box_pixel_height
        self.param_center_bbp_width = center_bounding_box_pixel_width
        self.param_bbp_height = bounding_box_pixel_height
        self.param_bbp_width = bounding_box_pixel_width
        self.param_bbp_area = self.param_bbp_height * self.param_bbp_width
        self.param_image_height = image_height
        self.param_image_width = image_width
        self.param_max_mouvement = max_mouvement
        self.param_min_mouvement = min_mouvement
        self.param_long_rotation = long_rotation
        self.param_speed_profile = speed_profile
        self.param_timeout = timeout

        self.timeout_pub = rospy.Publisher('/sonia_behaviors/timeout', MissionTimer, queue_size=5)

    def on_enter(self, userdata):
        self.vision_target_list = userdata.target_list_in
        self.position_reached = False
        self.alignement_reached = False

        self.average_x_pixel = 0.
        self.average_y_pixel = 0.
        self.average_width_pixel = 0.
        self.average_height_pixel = 0.
        self.average_area_pixel = 0.
        self.average_angle = 0.

        self.position_z = 0.3

        if userdata.camera_no == 2 or userdata.camera_no == 4 :
            self.cam_bottom = True
        else :
            self.cam_bottom = False

        self.start_time = time()
        Logger.log('Checking for position and alignement', Logger.REPORT_HINT)
        self.timeout_pub.publish(navUtils.missionTimerFunc("cross_check_vision_target", self.param_timeout, self.start_time, 1))
        self.vision_target = VisionTarget()
        self.get_position = rospy.Subscriber('/proc_nav/auv_states', Odometry, self.position_cb)
        
        if len(self.vision_target_list) == 0:
            return 'lost_target'
        else:
            self.compare()

    def position_cb(self, data):
        self.position_z = data.pose.pose.position.z

    def compare(self):
        sum_x = 0.
        sum_y = 0.
        sum_width = 0.
        sum_height = 0.
        sum_angle = 0.
        Logger.log(len(self.vision_target_list), Logger.REPORT_HINT)
        for i in range(len(self.vision_target_list)):
            Logger.log('Vision ' + str(i) + ' : ' + str(self.vision_target_list[i].x), Logger.REPORT_HINT)
            Logger.log('Vision ' + str(i) + ' : ' + str(self.vision_target_list[i].y), Logger.REPORT_HINT)
            Logger.log('Vision ' + str(i) + ' : ' + str(self.vision_target_list[i].width), Logger.REPORT_HINT)
            Logger.log('Vision ' + str(i) + ' : ' + str(self.vision_target_list[i].height), Logger.REPORT_HINT)
            Logger.log('Vision ' + str(i) + ' : ' + str(self.vision_target_list[i].angle), Logger.REPORT_HINT)
            
            sum_x += self.vision_target_list[i].x
            sum_y += self.vision_target_list[i].y
            sum_width += self.vision_target_list[i].width
            sum_height += self.vision_target_list[i].height
            sum_angle += self.vision_target_list[i].angle
            
            Logger.log('Sum x : ' + str(sum_x), Logger.REPORT_HINT)
            Logger.log('Sum y : ' + str(sum_y), Logger.REPORT_HINT)
            Logger.log('Sum width : ' + str(sum_width), Logger.REPORT_HINT)
            Logger.log('Sum height : ' + str(sum_height), Logger.REPORT_HINT)
            Logger.log('Sum angle : ' + str(sum_angle), Logger.REPORT_HINT)

        self.average_x_pixel = sum_x/len(self.vision_target_list)
        self.average_y_pixel = sum_y/len(self.vision_target_list)
        self.average_width_pixel = sum_width/len(self.vision_target_list)
        self.average_height_pixel = sum_height/len(self.vision_target_list)
        self.average_angle_pixel = sum_angle/len(self.vision_target_list)

        self.average_area_pixel = self.average_width_pixel*self.average_height_pixel
        
        Logger.log('Area of target (px): %f' %(self.average_height_pixel*self.average_width_pixel), Logger.REPORT_HINT)
        Logger.log('Area of bounding box (px): %f' %(self.param_bbp_height*self.param_bbp_width), Logger.REPORT_HINT)

        if self.average_area_pixel > self.param_bbp_area :
            self.position_reached = True
            Logger.log('Position reached', Logger.REPORT_HINT)
        else:
            self.position_reached = False

        Logger.log('x average: %f' %abs(self.average_x_pixel), Logger.REPORT_HINT)
        Logger.log(self.param_center_bbp_width/2, Logger.REPORT_HINT)
        Logger.log('y average: %f' %abs(self.average_y_pixel), Logger.REPORT_HINT)
        Logger.log(self.param_center_bbp_height/2, Logger.REPORT_HINT)

        if abs(self.average_y_pixel) <= self.param_center_bbp_height/2 and abs(self.average_x_pixel) <= self.param_center_bbp_width/2 :
            self.alignement_reached = True
            Logger.log('Alignement reached', Logger.REPORT_HINT)
        else:
            self.alignement_reached = False
        
        self.parse_data = True

    def align_front_with_vision(self):
        Logger.log('Alignement on target. Creating pose', Logger.REPORT_HINT)

        new_traj = MultiAddPose()
        new_pose = AddPose()
        
        if abs(self.average_x_pixel) <= (self.param_center_bbp_width/2):
            mouvement_x = 0
            Logger.log('X already aligned', Logger.REPORT_HINT)
        else: 
            mouvement_x = numpy.sign(self.average_x_pixel)*min(self.param_max_mouvement,abs(self.average_x_pixel/(self.average_width_pixel)))
        if abs(self.average_y_pixel) <= (self.param_center_bbp_height/2):
            mouvement_y = 0
            Logger.log('Y already aligned', Logger.REPORT_HINT)
        else: 
            mouvement_y = numpy.sign(self.average_y_pixel)*min(self.param_max_mouvement,abs(self.average_y_pixel/(self.average_height_pixel)))

        Logger.log('Déplacement y : %f' %mouvement_x, Logger.REPORT_HINT)
        Logger.log('Déplacement z : %f' %(-mouvement_y), Logger.REPORT_HINT)

        new_pose.position = Point(0., mouvement_x, -mouvement_y)
        new_traj.pose.append(self.fill_pose(new_pose))
        return (new_traj)

    def align_bottom_with_vision(self):
        Logger.log('Alignement on target. Creating pose', Logger.REPORT_HINT)

        new_traj = MultiAddPose()
        new_pose = AddPose()
        
        if abs(self.average_x_pixel) <= (self.param_center_bbp_width/2):
            mouvement_x = 0
            Logger.log('X already aligned', Logger.REPORT_HINT)
        else: 
            mouvement_x = numpy.sign(self.average_x_pixel)*min(self.param_max_mouvement,abs((self.average_x_pixel/self.param_image_width)/((self.average_area_pixel/self.param_bbp_area)*self.position_z)))
        if abs(self.average_y_pixel) <= (self.param_center_bbp_height/2):
            mouvement_y = 0
            Logger.log('Y already aligned', Logger.REPORT_HINT)
        else: 
            mouvement_y = numpy.sign(self.average_y_pixel)*min(self.param_max_mouvement,abs((self.average_y_pixel/self.param_image_height)/((self.average_area_pixel/self.param_bbp_area)*self.position_z)))

        Logger.log('Déplacement x : %f' %mouvement_x, Logger.REPORT_HINT)
        Logger.log('Déplacement y : %f' %mouvement_y, Logger.REPORT_HINT)

        new_pose.position = Point(mouvement_x, mouvement_y, 0.)
        new_traj.pose.append(self.fill_pose(new_pose))
        return (new_traj)
        
    def position_with_vision(self):
        Logger.log('Getting closer. Creating pose', Logger.REPORT_HINT)

        new_traj = MultiAddPose()
        new_pose = AddPose()

        mouvement = max(self.param_min_mouvement,(self.param_max_mouvement)*(1-((self.average_area_pixel)/(self.param_bbp_area))))

        if self.cam_bottom:
            new_pose.position = Point(0.,0.,mouvement)
            Logger.log('Déplacement z : %f' %mouvement, Logger.REPORT_HINT)
        else :
            Logger.log('Déplacement x : %f' %mouvement, Logger.REPORT_HINT)
            new_pose.position = Point(mouvement,0.,0.)

        new_traj.pose.append(self.fill_pose(new_pose))
        return (new_traj)

    def fill_pose(self, pose):
        return navUtils.addpose(pose.position.x, pose.position.y, pose.position.z, 0., 0., 0., 1, self.param_speed_profile, 0, self.param_long_rotation)


    def execute(self, userdata):
        actual = time()-self.start_time
        if self.parse_data == True:
            target_list = []
            userdata.target_list_out = target_list
            self.parse_data == False   
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
                return 'align'
            else:
                self.timeout_pub.publish(navUtils.missionTimerFunc("get_simple_vision_target", self.param_timeout, self.start_time, 4))
                return 'lost_target'
        if actual > self.param_timeout :
            self.timeout_pub.publish(navUtils.missionTimerFunc("get_simple_vision_target", self.param_timeout, self.start_time, 3))
            return 'search'

    def on_exit(self, userdata):
        pass