#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import MultiAddPose

class init_trajectory(EventState):

    '''
        Create the object for the trajectory

        #> trajectory           The object for the trajectory (MultiAddPose)           

        <= continue             Indicates that the object is created
    '''

    def __init__(self):
        
        super(init_trajectory, self).__init__(outcomes=['continue'],
                                              output_keys=['trajectory'])

    def execute(self, userdata):
        Logger.log('Creating a message for the trajectory', Logger.REPORT_HINT)
        userdata.trajectory = MultiAddPose()
        return 'continue'

    def on_exit(self, userdata):
        pass