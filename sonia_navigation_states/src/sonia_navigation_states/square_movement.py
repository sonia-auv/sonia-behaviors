#!/usr/bin/env python 
#-*- coding: utf-8 -*-

# standars includes
from time import time
import rospy
import math

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils

# flexbe includes
from flexbe_core import EventState, Logger
from sonia_common.msg import AddPose, MultiAddPose

class square_movement(EventState):

    '''
        This state allows the submarine to do a square path.

        -- boxX             uint8               Square width
        -- boxY             uint8               Square height

        ># input_traj       MultiAddPose        Input trajectory

        #> trajectory       MultiAddPose        Output trajectory

        <= continue                             End of the zigzag
    '''

    def __init__(self, boxX=1.0, boxY=1.0, speed=2):
        
        super(square_movement, self).__init__(outcomes=['continue'],
                                                     input_keys=['input_traj'],
                                                     output_keys=['trajectory'])

        self.boxX = boxX
        self.boxY = boxY
        self.speed = speed
        self.radius = 0.0

    def execute(self, userdata):

        traj = userdata.input_traj
        new_traj = MultiAddPose()
        # Force spline interpolation for circle
        new_traj.interpolation_method = 0      

        # Add previous waypoint if needed
        if not traj.pose:
            Logger.log('First position of the trajectory', Logger.REPORT_HINT)
        else:
            Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)            
            new_traj.pose = list(traj.pose)


        #first mouvement (move forward)
        new_traj.pose.append(navUtils.addpose(self.boxX/2, 0, 0, 0, 0, 0, self.speed, 0, self.radius, False))

        new_traj.pose.append(navUtils.addpose(0, self.boxY/2, 0, 0, 0, 0, self.speed, 0, self.radius, False))

        new_traj.pose.append(navUtils.addpose(-self.boxX, 0, 0, 0, 0, 0, self.speed, 0, self.radius, False))

        new_traj.pose.append(navUtils.addpose(0, -self.boxY, 0, 0, 0, 0, self.speed, 0, self.radius, False))

        new_traj.pose.append(navUtils.addpose(self.boxX, 0, 0, 0, 0, 0, self.speed, 0, self.radius, False))

        new_traj.pose.append(navUtils.addpose(0, self.boxY/2, 0, 0, 0, 0, self.speed, 0, self.radius, False))

        #last mouvement (move backward)
        new_traj.pose.append(navUtils.addpose(-self.boxX/2, 0, 0, 0, 0, 0, self.speed, 0, self.radius, False))

        # print debug
        Logger.log('Square movement has succesfully generated.', Logger.REPORT_HINT)

        userdata.trajectory = new_traj
        return 'continue'

    def on_exit(self, userdata):
        pass
