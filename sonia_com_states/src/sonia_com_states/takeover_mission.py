#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
import sonia_com_states.modules.com_utilities as comUtils
from time import sleep

from flexbe_core import EventState, Logger
from std_msgs.msg import Int8MultiArray
from sonia_common.msg import ModemUpdateMissionList

class takeover_mission(EventState):

    '''
        Takeover a specific mission that is not taken or has been failed

        -- mission_id           uint8   Position of the mission in the array
        -- has_com              bool    The submarine is using the underwater communication 

        <=takeover                      The mission can be taken since it has failed or not assigned
        <=no_takeover                   The mission can't be taken since it's not been attempt or no underwater communication
        <=already_done                  The mission can't be taken since it has been completed
    '''

    def __init__(self, mission_id=0, has_com=True):
        super(takeover_mission, self).__init__(outcomes=['takeover', 'no_takeover', 'assigned'])
        self.array = Int8MultiArray()
        self.other_array = Int8MultiArray()
        self.mission_id = mission_id
        self.has_com = has_com
        self.message_received = False
        self.message_received_other = False

        self.update_array = rospy.Publisher('/proc_underwater_com/mission_state_msg', ModemUpdateMissionList, queue_size=2)

    def mission_array_cb(self, msg):
        Logger.log('Received the updated state of the sub', Logger.REPORT_HINT)
        Logger.log(str(msg.data), Logger.REPORT_HINT)
        self.array = msg.data
        self.message_received = True

    def other_mission_array_cb(self, msg):
        Logger.log('Received the updated state of the other sub', Logger.REPORT_HINT)
        Logger.log(str(msg.data), Logger.REPORT_HINT)
        self.other_array = msg.data
        self.message_received_other = True

    def on_enter(self, userdata):
        self.receive_array = rospy.Subscriber('/proc_underwater_com/sub_mission_list', Int8MultiArray, self.mission_array_cb)
        self.other_receive_array = rospy.Subscriber('/proc_underwater_com/other_sub_mission_list', Int8MultiArray, self.other_mission_array_cb)
        
    def execute(self, userdata):
        if self.has_com == False:
            return 'no_takeover'

        if self.message_received == True and self.message_received_other == True:
            if self.array[self.mission_id] > 0:
                Logger.log('Mission already assigned to the submarine', Logger.REPORT_HINT)
                return 'assigned'
            elif self.other_array[self.mission_id] > 0:
                Logger.log('Mission is not yet failed or has been completed for the other submarine', Logger.REPORT_HINT)
                return 'no_takeover'
            else:
                Logger.log('Mission tranfered to the submarine', Logger.REPORT_HINT)
                self.update_array.publish(comUtils.update_mission_array(self.mission_id, 1))
                return 'takeover'

    def on_exit(self, userdata):
        self.receive_array.unregister()
        self.other_receive_array.unregister()