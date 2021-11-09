#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import sleep, time
from genpy.message import check_type
import rospy

from flexbe_core import EventState, Logger
from rospy.core import logerr
from geometry_msgs.msg import Pose

class set_initial_position(EventState):

    '''
        State to change the control mode.
        [...]
        
        -- mode             uint8       choose the control mode

        ># positionX        uint8       Input to the calculation function.

        #> output_value     object      The result of the calculation.

        <= done                         Indicates completion of the calculation.

        '''

    def __init__(self, positionX, positionY, positionZ, quaternionX, quaternionY, quaternionZ, quaternionW):
        
        super(set_initial_position, self).__init__(outcomes=['continue', 'failed'])
        #TO DO
        self.param_positionX 
        self.set_initial_position_pub = rospy.Publisher('/initial_condition', Pose, queue_size=2)

    def on_enter(self, userdata):

        self.set_initial_position_pub.publish(self.param_position)
        Logger.log('setting mode %s' %self.param_mode,Logger.REPORT_HINT)

    def execute(self, userdata):
        Logger.log('waiting 3 sec',Logger.REPORT_HINT)
        sleep(3)
        return 'continue'
        # TO DO : add handshake

    def on_exit(self, userdata):
        pass
