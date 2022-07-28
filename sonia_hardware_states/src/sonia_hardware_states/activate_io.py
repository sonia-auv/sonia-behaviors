#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import ActuatorDoAction, ActuatorSendReply, MissionTimer
from sonia_navigation_states.modules.navigation_utilities import missionTimerFunc

class activate_io(EventState):

    '''
        State to active the IOs of the submarine (arm not supported yet)

        -- element              uint8       {torpedo:0, dropper:1, arm:2}
        -- side                 uint8       {port:0, starboard:1} {arm_close:0, arm_open:1}
        -- action               uint8       {reset:0, launch/drop/exec:1}

        <= continue                         Activation of the element successful
        <= failed                           Failed to activate the element (Most likely due to kill switch being on)
        <= timed out                        Activation timed out

    '''

    def __init__(self, element, side, action, timeout=8):
        super(activate_io, self).__init__(outcomes=['continue', 'failed', 'timeout'])
        
        self.time_start = None
        self.element = element
        self.side = side
        self.action = action
        self.timeout = timeout
        self.response = ''

        self.action_pub = rospy.Publisher('/provider_actuators/do_action_to_actuators', ActuatorDoAction, queue_size=2)
        self.timeout_pub = rospy.Publisher('/sonia_behaviors/timeout', MissionTimer, queue_size=5)

    def on_enter(self, userdata):
        self.action_sub = rospy.Subscriber('/provider_actuators/do_action_from_actuators', ActuatorSendReply, self.reply_cb)
        self.action_pub.publish(ActuatorDoAction(self.element, self.side, self.action))
        self.time_start = time()
        self.timeout_pub.publish(missionTimerFunc(f"activate_io (element{self.element}, side{self.side})", self.timeout, str(self.time_start), 1))
        Logger.log(f'Action : {self.element} is launched', Logger.REPORT_HINT)

    def reply_cb(self, data):
        if data.element == self.element and self.side == data.side:
            if data.response == ActuatorSendReply.RESPONSE_FAILURE:
                self.timeout_pub.publish(missionTimerFunc("activate_io (element{self.element}, side{self.side})", self.timeout, str(self.time_start), 4))
                self.response = 'failed'
            elif data.response == ActuatorSendReply.RESPONSE_SUCCESS:
                self.timeout_pub.publish(missionTimerFunc("activate_io (element{self.element}, side{self.side})", self.timeout, str(self.time_start), 2))
                self.response = 'continue'
            elif data.response == ActuatorSendReply.RESPONSE_TIMED_OUT:
                self.timeout_pub.publish(missionTimerFunc("activate_io (element{self.element}, side{self.side})", self.timeout, str(self.time_start), 3))
                self.response = 'timeout'
            else:
                Logger.log(f"/provider_actuators/do_action_from_actuators sent a message with the response code {data.response} which should not exist", Logger.REPORT_ERROR)

    def execute(self, userdata):
        if self.response != "":
            return self.response
        if time() - self.time_start >= self.timeout:
            self.timeout_pub.publish(missionTimerFunc("activate_io (element{self.element}, side{self.side})", self.timeout, str(self.time_start), 3))
            Logger.log('Timeout Reached', Logger.REPORT_HINT)
            return 'timeout'

    def end(self, userdata):
        self.action_sub.unregister()