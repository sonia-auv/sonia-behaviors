#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from sonia_common.srv import ExecuteCmd

class stop_filter_chain(EventState):
    '''
        Stop the filterchain started before
        [...]

        ># filterchain          string      Filterchain to stop
        ># camera_no            string      Camera to stop

        <= continue			Indicates that the camera started
        <= failed			Indicates that the camera didn't started

    '''
    def __init__(self):
        super(stop_filter_chain, self).__init__(   outcomes=['continue', 'failed'],
                                                    input_keys=['filterchain', 'camera_no'])
        self.execute_vision_cmd = None
        
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
            if userdata.camera_no == 1:
                self.param_media_name = '/camera_array/front/image_raw/compressed'
            elif userdata.camera_no == 2:
                self.param_media_name = '/camera_array/bottom/image_raw/compressed'
            elif userdata.camera_no == 3:
                self.param_media_name = '/proc_simulation/front'
            elif userdata.camera_no == 4:
                self.param_media_name = '/proc_simulation/bottom'
            else:
                rospy.loginfo('Bad camera no, range is 1-4')
                self.result = 'failed'
                return 'failed'

            self.execute_vision_cmd(userdata.filterchain,
                                    userdata.filterchain,
                                    self.param_media_name,
                                    2)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            self.result = 'failed'
            return 'failed'

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        if self.result == 'failed' :
            Logger.log('Error in start_filter_chain state, more info in console', Logger.REPORT_HINT)
        else:
            Logger.log('Filter chain stopped : %s' %userdata.filterchain, Logger.REPORT_HINT)
        pass
 
