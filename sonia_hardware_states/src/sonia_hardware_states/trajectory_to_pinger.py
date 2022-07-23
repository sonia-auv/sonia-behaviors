#!/usr/bin/env python 
#-*- coding: utf-8 -*-

# standars includes
from time import time
import rospy

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils

from flexbe_core import EventState, Logger
from sonia_common.msg import MultiAddPose

class trajectory_to_pinger(EventState):

    '''
        Create pose to reache the pingers
        Same as manual_add_pose_to_trajectory but takes input keys as parameter
        '''

    def __init__(self, speed=0, precision=0, long_rotation=False):
        
        super(trajectory_to_pinger, self).__init__(outcomes=['continue'],
                                                     input_keys=['input_traj', 'real_angle','distance'],
                                                     output_keys=['trajectory'])
        # create addpose object
        self.speed = speed
        self.precision = precision
        self.long_rotation = long_rotation
        self.pose = navUtils.addpose(0, 0, 0, 0, 0, 0, 1, self.speed, self.precision, self.long_rotation)

    def execute(self, userdata):
        self.rotation = navUtils.addpose(0, 0, 0, 0, 0, userdata.real_angle, 1, self.speed, self.precision, self.long_rotation)
        self.translation = navUtils.addpose(userdata.distance, 0, 0, 0, 0, 0, 1, self.speed, self.precision, self.long_rotation)
        traj = userdata.input_traj
        new_traj = MultiAddPose()
        if not traj.pose:
            Logger.log('First position of the trajectory', Logger.REPORT_HINT)
        else:
            Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)            
            new_traj.pose = list(traj.pose)

        new_traj.pose.append(self.rotation)
        new_traj.pose.append(self.translation)
        userdata.trajectory = new_traj
        return 'continue'

    def on_exit(self, userdata):
        pass