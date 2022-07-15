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

        -- filterchain          string      Detection task
        -- target               string      Header name to filter result
        -- camera_no            uint8       Enter 1:Front 2:Bottom 3:Front simulation 4:Bottom simulation
        -- param_cmd            uint8       Enter 1:Open  2:Close

        <= continue			Indicates that the camera started
        <= failed			Indicates that the camera didn't started

    '''
    def __init__(self, filterchain, target, camera_no, param_cmd):
        super(start_filter_chain, self).__init__(   outcomes=['continue', 'failed'],
                                                    output_keys=['filterchain', 'camera_no', 'target'])
        self.execute_vision_cmd = None
        self.camera_no = camera_no
        self.filterchain = filterchain
        self.param_target = target
        self.param_cmd = param_cmd
        
    def on_enter(self, userdata):
        self.result = ""
        try:
            rospy.wait_for_service('/proc_image_processing/execute_cmd')
            self.execute_vision_cmd = rospy.ServiceProxy('/proc_image_processing/execute_cmd', ExecuteCmd)
        except rospy.ServiceException as exc:
            rospy.loginfo('Cannot access service' + str(exc))
            self.result = 'failed'
            return 'failed'
        try:
            if self.camera_no == 1:
                self.param_media_name = '/camera_array/front/image_raw/compressed'
            elif self.camera_no == 2:
                self.param_media_name = '/camera_array/bottom/image_raw/compressed'
            elif self.camera_no == 3:
                self.param_media_name = '/proc_simulation/front'
            elif self.camera_no == 4:
                self.param_media_name = '/proc_simulation/bottom'
            else:
                rospy.loginfo('Bad camera no, range is 1-4')
                self.result = 'failed'
                return 'failed'

            if self.param_cmd != 1 and self.param_cmd != 2:
                rospy.loginfo('Bad command, range is 1-2')
                self.result = 'failed'
                return 'failed'

            self.execute_vision_cmd(self.filterchain,
                                    self.filterchain,
                                    self.param_media_name,
                                    self.param_cmd)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            self.result = 'failed'
            return 'failed'

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        userdata.filterchain = '/proc_image_processing/' + self.filterchain + '_result'
        userdata.camera_no = self.camera_no
        userdata.target = self.param_target
        if self.result == 'failed' :
            Logger.log('Error in start_filter_chain state, more info in console', Logger.REPORT_HINT)
        else:
            if self.param_cmd == 1:
                Logger.log('Filter chain started : %s' %userdata.filterchain, Logger.REPORT_HINT)
            elif self.param_cmd == 2:
                Logger.log('Filter chain stopped : %s' %userdata.filterchain, Logger.REPORT_HINT)
        pass
 
