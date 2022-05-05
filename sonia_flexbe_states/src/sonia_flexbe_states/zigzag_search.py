#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy
import math

from flexbe_core import EventState, Logger
from sonia_common.msg import AddPose, MultiAddPose

class zizag_search(EventState):

    '''
    This state generate a zigzag serach trajectory
    '''
    '''
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
                                 _|_        ^ x
                                |   |       |
                               _|   |_      |
                              |_ sub _|     -----> y
                                |   |       body frame
                                |___|

     Side false = start to left, true = start to right

     radius can roud the trajectory.
        '''

    def __init__(self, boxX=5, boxY=5, stroke=1 , radius=0.4, side = False ):
        
        super(zizag_search, self).__init__(outcomes=['continue'],
                                                     input_keys=['input_traj'],
                                                     output_keys=['trajectory'])

        self.boxX= boxX
        self.boxY = boxY
        self.stroke =stroke
        self.radius = radius
        self.side =side
        # Compute trajectory parameters
        self.fullStep = math.floor(self.boxX/self.stroke)
        self.residue = self.boxX % self.stroke



    def execute(self, userdata):

        traj = userdata.input_traj
        new_traj = MultiAddPose()
       
        # Add previous waypoint if needed
        if not traj.pose:
            Logger.log('First position of the trajectory', Logger.REPORT_HINT)
        else:
            Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)            
            new_traj.pose = list(traj.pose)

        #first mouvement (1/2 stroke)
        new_traj.pose.append(self.addpose(0, self.get_direction()*(self.boxY/2), 0, 0, 0, 0, 1, 0, self.radius,False))

        # Generate point for 
        for i in range(self.fullStep):

            # move sub foward
            new_traj.pose.append(self.addpose(self.stroke, 0, 0, 0, 0, 0, 1, 0, self.radius,False))

            # move sub sideway
            new_traj.pose.append(self.addpose(0, self.get_direction()*(self.boxY), 0, 0, 0, 0, 1, 0, self.radius,False))

        # add resudue point if needed
        if self.residue > 0 :
            
            i+=1
            # move sub foward
            new_traj.pose.append(self.addpose(self.residue, 0, 0, 0, 0, 0, 1, 0, self.radius,False))
            # move sub sideway
            new_traj.pose.append(self.addpose(0, self.get_direction()*(self.boxY), 0, 0, 0, 0, 1, 0, self.radius,False))

        # print debug
        Logger.log('Zigzag serach as succesfully generated '+ str(i) + 'waypoints' , Logger.REPORT_HINT)

        userdata.trajectory = new_traj
        return 'continue'

    def addpose(self, x, y, z, rx, ry,rz, frame, speed, fine, rot):
        buffer = AddPose()
        buffer.position.x = x
        buffer.position.y = y
        buffer.position.z = z
        buffer.orientation.x = rx
        buffer.orientation.y = ry
        buffer.orientation.z = rz
        buffer.frame = frame
        buffer.speed = speed
        buffer.fine = fine
        buffer.rotation = rot

        return buffer

    def get_direction(self):
        if self.side:
            signe = 1
        else:
            signe = -1

        self.side = not self.side

        return signe

    def on_exit(self, userdata):
        pass