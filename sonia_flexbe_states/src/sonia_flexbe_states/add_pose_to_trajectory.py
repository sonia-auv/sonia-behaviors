#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import AddPose, MultiAddPose

class add_pose_to_trajectory(EventState):

    '''
        Create the object for the trajectory
        
        -- positionX        uint8           Movement in X axis
        -- positionY        uint8           Movement in Y axis
        -- positionZ        uint8           Movement in Z axis
        -- orientationX     uint8           Rotation in roll axis
        -- orientationY     uint8           Rotation in pitch axis
        -- orientationZ     uint8           Rotation in yaw axis
        -- frame            uint8           0 : Absolute position and absolute angle
                                                1 : Relative position and relative angle
                                                2 : Relative position and absolute angle
                                                3 : Absolute position and relative angle 
        -- speed            uint8           1 : Fast speed
                                                2 : Low speed
        -- precision        float64         Precision of the movement
        -- path             bool            False : Quickest rotation
                                                True : Follow the rotation 
        
        ># input_traj       MultiAddPose    Input trajectory

        #> trajectory       MultiAddPose    Output trajectory with the new pose added

        <= continue                         Indicates completion.
        '''

    def __init__(self, positionX, positionY, positionZ, orientationX, orientationY, orientationZ, frame, speed=1, precision=0, rotation=True):
        
        super(add_pose_to_trajectory, self).__init__(outcomes=['continue'],
                                                     input_keys=['input_traj'],
                                                     output_keys=['trajectory'])

        self.pose = AddPose()
        self.pose.position.x = positionX
        self.pose.position.y = positionY
        self.pose.position.z = positionZ
        self.pose.orientation.x = orientationX
        self.pose.orientation.y = orientationY
        self.pose.orientation.z = orientationZ
        self.pose.frame = frame
        self.pose.speed = speed
        self.pose.fine = precision
        self.pose.rotation = rotation

    def execute(self, userdata):
        traj = userdata.input_traj
        new_traj = MultiAddPose()
        if not traj.pose:
            Logger.log('First position of the trajectory', Logger.REPORT_HINT)
        else:
            Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)            
            new_traj.pose = list(traj.pose)
        new_traj.pose.append(self.pose)
        userdata.trajectory = new_traj
        return 'continue'

    def on_exit(self, userdata):
        pass