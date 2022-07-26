#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import ActuatorDoAction

class activate_io(EventState):

    '''
        State to active the IOs of the submarine (arm not supported yet)

        -- element              uint8       see ActuatorDoAction message
        -- side                 uint8       see ActuatorDoAction message
        -- action               uint8       see ActuatorDoAction message

        <= continue                         Activation of the element successful
        <= failed                           Failed to activate the element (service issue)

    '''

    def __init__(self, element, side, action, timeout=10):
        super(activate_io, self).__init__(outcomes=['continue', 'failed'])
        
        self.start_time = None
        self.param_side = side
        self.param_element = element
        self.param_action = action
        self.timeout = timeout

        self.action_pub = rospy.Publisher('/provider_actuators/do_action_to_actuators', ActuatorDoAction, queue_size=2)

    def on_enter(self, userdata):
        self.action_pub.publish(ActuatorDoAction(self.param_element, self.param_side, self.param_action))
        

    def execute(self, userdata):
        rospy.loginfo('Action : %i is launch' % int(self.param_element))
        return 'continue'

    def end(self, userdata):
        pass