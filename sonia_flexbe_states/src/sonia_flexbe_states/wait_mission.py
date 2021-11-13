#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import MissionSwitchMsg

class wait_mission(EventState):

    '''
        Wait for the mission switch to start the missions.

    '''

    def __init__(self):
        
        super(wait_mission, self).__init__(outcomes=['continue', 'failed'])

        self.get_mission_state_sub = rospy.Subscriber('/provider_kill_mission/mission_switch_msg', MissionSwitchMsg, self.get_mission_state_cb)

    def get_mission_state_cb(self, data):
        self.mission_state = data.state

    def on_enter(self, userdata):
        self.mission_state = False

    def execute(self, userdata):
        if self.mission_state == True:
            Logger.log('Mission Switch activated', Logger.REPORT_HINT)
            return 'continue'

    def on_exit(self, userdata):
        pass
