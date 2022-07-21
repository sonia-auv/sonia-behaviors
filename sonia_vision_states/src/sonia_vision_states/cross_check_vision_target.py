#!/usr/bin/env python 
#-*- coding: utf-8 -*-
from time import time
import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import VisionTarget, MissionTimer
from sonia_navigation_states.modules.navigation_utilities import missionTimerFunc

class cross_check_vision_target(EventState):

    '''
        Verify that the vision target has been found

        -- number_target            uint8           Number of target to cross-check
        -- timeout                  uint8           Time to stop looking at this position
        -- ai_confidence_target     uint8           Confidence required to continue

        ># vision_target_list       VisionTarget    List of VisionTarget
        #> input_trajectory         MultiAddPose    Input tajectory

        #> output_trajectory        MultiAddPose    Output tajectory

        <= continue                                 Confidence reached a satisfying level
        <= failed                                   Continue to search for the target
    
    '''

    def __init__(self, number_samples, timeout):

        super(cross_check_vision_target, self).__init__(outcomes = ['continue', 'failed'],
                                                        input_keys = ['AI_target', 'SIFT_target', 'conv_target'],
                                                        output_key = ['confidence', 'target_position'])
        
        self.param_number_samples = number_samples
        self.param_timeout = timeout
        self.timeout_pub = rospy.Publisher('/sonia_behaviors/timeout', MissionTimer, queue_size=5)

    def vision_cb(self, data):
        self.number_of_found += 1

    def on_enter(self, userdata): 
        self.number_of_found = 0
        self.get_vision_data = rospy.Subscriber(userdata.filterchain, VisionTarget, self.vision_cb)
        self.start_time = time()
        Logger.log('Checking to find the target for %d seconds' %self.param_timeout, Logger.REPORT_HINT)
        self.timeout_pub.publish(missionTimerFunc("cross_check_vision_target", self.param_timeout, self.start_time, 1))

    def execute(self, userdata):
        actual = time()-self.start_time       
        if self.number_of_found > self.param_number_samples:
            self.timeout_pub.publish(missionTimerFunc("cross_check_vision_target", self.param_timeout, self.start_time, 2))
            Logger.log('Target found', Logger.REPORT_HINT)
            return 'continue'
        if actual > self.param_timeout:
            self.timeout_pub.publish(missionTimerFunc("cross_check_vision_target", self.param_timeout, self.start_time, 3))
            Logger.log('No target found', Logger.REPORT_HINT)
            return 'failed'

    def on_exit(self, userdata):
        self.get_vision_data.unregister()