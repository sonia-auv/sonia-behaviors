#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
import sonia_com_states.modules.com_utilities as comUtils

from flexbe_core import EventState, Logger
from sonia_common.msg import ModemUpdateMissionList

class send_update(EventState):

    '''
        Send the updated state for a single mission

        -- mission          uint8   Position in the mission array to change the state
        -- state            int8    Value of the state to change

        <=continue                  Message sent to the underwater_com
    '''

    def __init__(self, mission, state):
        super(send_update, self).__init__(outcomes=['continue'])

        self.mission = mission
        self.state = state
        
        self.mission_pub = rospy.Publisher('/proc_underwater_com/mission_state_msg', ModemUpdateMissionList, queue_size=2)

    def on_enter(self, userdata):
        Logger.log('Updating mission ' + str(self.mission) + ' with the state ' + str(self.state), Logger.REPORT_HINT)
        self.mission_pub.publish(comUtils.update_mission_array(self.mission, self.state))

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass