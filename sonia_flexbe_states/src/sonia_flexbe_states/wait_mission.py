#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import sleep, time
from genpy.message import check_type
import rospy

from flexbe_core import EventState, Logger
from rospy.core import logerr
from sonia_common.msg import MissionSwitchMsg
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry

class wait_mission(EventState):

    '''
        Wait for the mission switch to start the missions.

        '''

    def __init__(self, topic):
        
        super(wait_mission, self).__init__(outcomes=['continue', 'failed'])

        self.get_mission_state_sub = rospy.Subscriber('/provider_kill_mission/mission_switch_msg', MissionSwitchMsg, self.get_mission_state_cb)

    def get_mission_state_cb(self, data):
        self.mission_state = data.data

    def on_enter(self, userdata):
        self.mission_state = False

    def execute(self, userdata):
        if self.mission_state == True:
            return 'continue'

    def on_exit(self, userdata):
        pass
