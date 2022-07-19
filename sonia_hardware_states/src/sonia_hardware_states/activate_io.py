#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from sonia_common.srv import ActuatorDoActionSrv, ActuatorDoActionSrvRequest

class activate_io(EventState):

    '''
        State to active the IOs of the submarine (arm not supported yet)

        -- element              uint8       1 : torpedos
                                            2 : droppers
                                            3 : arm
        -- side                 uint8       1 : port
                                            2 : starboard

        <= continue                         Activation of the element successful
        <= failed                           Failed to activate the element (service issue)

    '''

    def __init__(self, element, side, timeout=10):
        super(activate_io, self).__init__(outcomes=['continue', 'failed'])
        
        self.start_time = None
        self.do_action = None
        self.param_side = side
        self.param_element = element
        self.timeout = timeout

        self.action = ActuatorDoActionSrvRequest()
        self.action_side = {'1': self.action.SIDE_PORT, '2': self.action.SIDE_STARBOARD}
        self.action_element = {'1': self.action.ELEMENT_TORPEDO, '2': self.action.ELEMENT_DROPPER, '3': self.action.ELEMENT_ARM}

    def on_enter(self, userdata):
        try:
            rospy.wait_for_service('/proc_actuators/cm_action_srv', self.timeout)
            self.do_action = rospy.ServiceProxy('/proc_actuators/cm_action_srv', ActuatorDoActionSrv)
            # rospy.wait_for_service('/provider_actuators/do_action_srv', self.timeout)
            # self.do_action = rospy.ServiceProxy('/provider_actuators/do_action_srv', ActuatorDoActionSrv)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service is not available : ' + str(exc))
            return 'failed'

        try:
            self.do_action(self.action_element[str(int(self.param_element))], self.action_side[str(int(self.param_side))], self.action.ACTION_DROPPER_LAUNCH)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request : ' + str(exc))
            return 'failed'

    def execute(self, userdata):
        rospy.loginfo('Action : %i is launch' % int(self.param_element))
        return 'continue'

    def end(self, userdata):
        pass