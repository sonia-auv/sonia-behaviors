#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from sonia_common.srv import ImuTareSrv

class imu_tare(EventState):

    '''
        State to tare the IMU

        <= continue                         Activation successful
        <= failed                           Failed to call the service

    '''

    def __init__(self):
        super(imu_tare, self).__init__(outcomes=['continue', 'failed'])

    def on_enter(self, userdata):
        rospy.wait_for_service('/provider_imu/tare')
        self.tare = rospy.ServiceProxy('/provider_imu/tare', ImuTareSrv)
        try:
            self.tare({})

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            return 'failed'

    def execute(self, userdata):
        Logger.log('Tare completed', Logger.REPORT_HINT)
        return 'continue'

    def end(self, userdata):
        pass