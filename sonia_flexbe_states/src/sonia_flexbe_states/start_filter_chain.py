#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import logging
import rospy

from flexbe_core import EventState, Logger
from sonia_common.srv import ExecuteCmd

class start_filter_chain(EventState):
    '''
        Start the filterchain entered
        [...]

        -- param_node_name      string      Detection task
        -- header_name          string      Header name to filter result
        -- camera_no            uint8       Enter 1:Front 2:Bottom 3:Front simulation 4:Bottom simulation
        -- param_cmd            uint8       Enter 1:Open  2:Close

        <= continue			Indicates that the camera started
        <= failed			Indicates that the camera didn't started

    '''
    def __init__(self, param_node_name, header_name, camera_no, param_cmd):
        super(start_filter_chain, self).__init__(   outcomes=['continue', 'failed'],
                                                    output_keys=['filterchain', 'camera_no', 'header_name'])
        self.execute_vision_cmd = None
        self.camera_no = camera_no
        self.param_node_name = param_node_name
        self.param_header_name = header_name
        self.param_cmd = param_cmd
        
    def on_enter(self, userdata):
        try:
            rospy.wait_for_service('/proc_image_processing/execute_cmd')
            self.execute_vision_cmd = rospy.ServiceProxy('/proc_image_processing/execute_cmd', ExecuteCmd)
        except rospy.ServiceException as exc:
            rospy.loginfo('Cannot access service' + str(exc))
            return 'failed'
        try:
            if self.camera_no == 1:
                self.param_media_name = '/camera_array/front/image_raw/compressed'
            elif self.camera_no == 2:
                self.param_media_name = '/camera_array/bottom/image_raw/compressed'
            elif self.camera_no == 3:
                self.param_media_name = '/front_simulation'
            else:
                self.param_media_name = '/bottom_simulation'

            self.execute_vision_cmd(self.param_node_name,
                                    self.param_node_name,
                                    self.param_media_name,
                                    self.param_cmd)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            return 'failed'

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        userdata.filterchain = '/proc_image_processing/' + self.param_node_name + '_result'
        userdata.camera_no = self.camera_no
        userdata.header_name = self.param_header_name
        Logger.log('Filter chain started : %s' %userdata.filterchain, Logger.REPORT_HINT)
        pass
 
