#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from flexbe_core import EventState, Logger
from sonia_common import AddPose

class add_pose_to_trajectory(EventState):

    '''
        Create the object for the trajectory
        
        -- positionX        uint8       The function that performs [...]
        -- positionY        uint8       The function that performs [...]
        -- positionZ        uint8       The function that performs [...]
        -- orientationX     uint8       The function that performs [...]
        -- orientationY     uint8       The function that performs [...]
        -- orientationZ     uint8       The function that performs [...]
        -- frame            uint8       0 : Absolute position and absolute angle
                                            1 : Relative position and relative angle
                                            2 : Relative position and absolute angle
                                            3 : Absolute position and relative angle 
        -- time             uint8       The function that performs [...]
        -- precision        float64     The function that performs [...]
        -- path             bool        The function that performs [...]
        
        <= continue                     Indicates completion.
        '''

    def __init__(self, positionX, positionY, positionZ, orientationX, orientationY, orientationZ, frame, time=5, precision=0, rotation=True):
        
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
        self.pose.speed = time
        self.pose.fine = precision
        self.pose.rotation = rotation

    def execute(self, userdata):
        Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)
        input_traj = userdata.input_traj
        input_traj.push_back(self.pose)
        userdata.trajectory = input_traj        
        return 'continue'

    def on_exit(self, userdata):
        pass