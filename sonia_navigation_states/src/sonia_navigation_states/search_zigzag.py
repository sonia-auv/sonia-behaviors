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

class search_zigzag(EventState):

    '''
        This state generate a zigzag serach trajectory
                |<------------- box Y ---------------->|
            _ _  ______________________________________
                ^  |
                |  |
                |  |______________________________________  _ _
                |                                         |  ^
            box X                                       |  stroke 
                |   ______________________________________| _v_
                |  |
                |  |
            _v_ |___________________
                                    ___        ^ x
                                   | ^ |       |
                                  _|   |_      |
                                 |_ sub _|     -----> y
                                   |   |       body frame
                                   |___|

        -- boxX             uint8               Length of zigzag
        -- boxY             uint8               Width of zigzag
        -- stroke           float               Distance between changes of direction
        -- side             bool                False = start to left, True = start to right

        ># input_traj       MultiAddPose        Input trajectory

        #> trajectory       MultiAddPose        Output trajectory

        <= continue                             End of the zigzag
    '''

    def __init__(self, boxX=5, boxY=5, stroke=0.8 , side = False ):
        
        super(search_zigzag, self).__init__(outcomes=['continue'],
                                                     input_keys=['input_traj'],
                                                     output_keys=['trajectory'])

        self.boxX = boxX
        self.boxY = boxY
        self.stroke = stroke
        self.radius = 0
        self.side = side

        # Compute trajectory parameters
        self.fullStep = int(math.floor(self.boxX/self.stroke))
        self.residue = self.boxX % self.stroke

    def execute(self, userdata):

        traj = userdata.input_traj
        new_traj = MultiAddPose()
        # Force spline interpolation for snake
        new_traj.interpolation_method = 2
       
        # Add previous waypoint if needed
        if not traj.pose:
            Logger.log('First position of the trajectory', Logger.REPORT_HINT)
        else:
            Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)            
            new_traj.pose = list(traj.pose)

        #first mouvement (1/2 stroke)
        new_traj.pose.append(navUtils.addpose(0, self.get_direction()*(self.boxY/2), 0, 0, 0, 0, 1, 0, self.radius,False))

        # Generate point for 
        for i in range(self.fullStep):

            # move sub foward
            new_traj.pose.append(navUtils.addpose(self.stroke, 0, 0, 0, 0, 0, 1, 0, self.radius,False))

            # move sub sideway
            new_traj.pose.append(navUtils.addpose(0, self.get_direction()*(self.boxY), 0, 0, 0, 0, 1, 0, self.radius,False))

        # add resudue point if needed
        if self.residue > 0 :
            
            i+=1
            # move sub foward
            new_traj.pose.append(navUtils.addpose(self.residue, 0, 0, 0, 0, 0, 1, 0, self.radius,False))
            # move sub sideway
            new_traj.pose.append(navUtils.addpose(0, self.get_direction()*(self.boxY), 0, 0, 0, 0, 1, 0, self.radius,False))

        # print debug
        Logger.log('Zigzag search has succesfully generated ' + str(i) + ' waypoints', Logger.REPORT_HINT)

        userdata.trajectory = new_traj
        return 'continue'

    def get_direction(self):
        if self.side:
            signe = 1
        else:
            signe = -1

        self.side = not self.side

        return signe

    def on_exit(self, userdata):
        pass
