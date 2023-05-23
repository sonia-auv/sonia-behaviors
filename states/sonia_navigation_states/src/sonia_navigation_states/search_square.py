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

class search_square(EventState):

    '''
        This state generate a circle serach trajectory inside a square
                    |<----------------- box size ----------------->|
            _ _      ______________________________________________
                ^   |      
                |   |      
                |   |       _______________________________________
                |   |      |                                       |
                |   |      |                                       |
                |   |      |       _________________________       |
                |   |      |      |         ^               |      |
                |   |      |      |         |               |      |
                |   |      |      |         |stroke         |      |
                |   |      |      |        _v_              |      |     ^ x
                |   |      |      |       | ^ |             |      |     |
                |   |      |      |      _|   |_            |      |     |
        box size    |      |      |     |_ sub _|----       |      |     -----> y
                |   |      |      |       |   |      |      |      |     body frame
                |   |      |      |       |___|      |      |      |
                |   |      |      |                  |      |      |
                |   |      |      |                  |      |      |
                |   |      |      |__________________|      |      |
                |   |      |                                |      |
                |   |      |                                |      |
                |   |      |________________________________|      |
                |   |                                              |
                |   |                                              |
            _ _ v   |______________________________________________|


        -- box_size         uint8               Length of box
        -- stroke           float               Distance between changes of direction
        -- side             bool                False = start to left, True = start to right

        ># input_traj       MultiAddPose        Input trajectory

        #> trajectory       MultiAddPose        Output trajectory

        <= continue                             End of the zigzag
    '''

    def __init__(self, box_size=5, stroke=0.8, side = False ):
        
        super(search_square, self).__init__(outcomes=['continue'],
                                                     input_keys=['input_traj'],
                                                     output_keys=['trajectory'])

        self.box_size = box_size
        self.stroke = stroke
        self.radius = 0.0
        self.side = side
        self.signe = 1

        # Compute trajectory parameters
        self.fullStep = int(math.floor(self.box_size/self.stroke))
        self.residue = self.box_size % self.stroke

    def execute(self, userdata):

        traj = userdata.input_traj
        new_traj = MultiAddPose()
        # Force spline interpolation for circle
        new_traj.interpolation_method = 2       

        # Add previous waypoint if needed
        if not traj.pose:
            Logger.log('First position of the trajectory', Logger.REPORT_HINT)
        else:
            Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)            
            new_traj.pose = list(traj.pose)


        #second mouvement (1/2 stroke)
        #new_traj.pose.append(navUtils.addpose(0, self.get_direction()*(self.boxY/2), 0, 0, 0, 0, 1, 0, self.radius,False))

        # Generate point for 
        for i in range(self.fullStep):

            # move sub foward
            new_traj.pose.append(navUtils.addpose(self.signe*(i+1)*self.stroke, 0, 0, 0, 0, 0, 1, 0, self.radius,False))

            # move sub sideway
            new_traj.pose.append(navUtils.addpose(0, self.signe*(i+1)*self.stroke, 0, 0, 0, 0, 1, 0, self.radius,False))

            self.change_direction()

        # add residue point if needed
        if self.residue > 0 :
            
            i+=1
            # move sub foward
            new_traj.pose.append(navUtils.addpose(self.signe*self.residue, 0, 0, 0, 0, 0, 1, 0, self.radius,False))
            # move sub sideway
            new_traj.pose.append(navUtils.addpose(0, self.signe*self.residue, 0, 0, 0, 0, 1, 0, self.radius,False))

        # print debug
        Logger.log('Zigzag search has succesfully generated ' + str(i) + ' waypoints', Logger.REPORT_HINT)

        userdata.trajectory = new_traj
        return 'continue'

    def change_direction(self):
        if self.side:
            self.signe = 1
        else:
            self.signe = -1

        self.side = not self.side

    def on_exit(self, userdata):
        pass
