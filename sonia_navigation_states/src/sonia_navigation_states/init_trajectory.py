#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import MultiAddPose

class init_trajectory(EventState):

    '''
        Create the object for the trajectory list

        -- interpolation_method     uint8   Interpolation method for the trajectory
                                            0 : Hermite
                                            1 : v5cubic
                                            2 : Spline

        #> trajectory           The object for the trajectory (MultiAddPose)           

        <= continue             Indicates that the object is created
    '''

    def __init__(self,interpolation_method=0):
        
        super(init_trajectory, self).__init__(outcomes=['continue'],
                                              output_keys=['trajectory'])
        self.interpolation_method = interpolation_method


    def execute(self, userdata):
        Logger.log('Creating a message for the trajectory', Logger.REPORT_HINT)
        userdata.trajectory = MultiAddPose()
        userdata.trajectory.interpolation_method = self.interpolation_method 
        return 'continue'

    def on_exit(self, userdata):
        pass
