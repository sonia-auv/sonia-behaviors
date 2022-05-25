#!/usr/bin/env python 
#-*- coding: utf-8 -*-

# standars includes
from time import time
import rospy

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils

from flexbe_core import EventState, Logger
from sonia_common.msg import MultiAddPose

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
        -- speed            uint8           0 : Normal speed
                                                1 : Fast speed
                                                2 : Low speed
        -- precision        float64         Precision of the movement
        -- long_rotation    bool            False : Quickest rotation
                                                True : Follow the rotation 
        
        ># input_traj       MultiAddPose    Input trajectory

        #> trajectory       MultiAddPose    Output trajectory with the new pose added

        <= continue                         Indicates completion.
        '''

    def __init__(self, positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False):
        
        super(add_pose_to_trajectory, self).__init__(outcomes=['continue'],
                                                     input_keys=['input_traj'],
                                                     output_keys=['trajectory'])
        # create addpose object
        self.pose = navUtils.addpose(positionX, positionY, positionZ, orientationX, orientationY, orientationZ,frame, speed, precision, long_rotation)

    def execute(self, userdata):
        traj = userdata.input_traj
        new_traj = MultiAddPose()
        if not traj.pose:
            Logger.log('First position of the trajectory', Logger.REPORT_HINT)
        else:
            Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)            
            new_traj.pose = list(traj.pose)

        Logger.log('Pose : x = '+ str(self.pose.position.x) + ', y = ' + str(self.pose.position.y) + ' z = ' + str(self.pose.position.z) + \
            ' roll = ' + str(self.pose.orientation.x) + ' pitch = ' + str(self.pose.orientation.y) + ' yaw = ' + str(self.pose.orientation.z) + \
            ' frame = ' + str(self.pose.frame), Logger.REPORT_HINT)

        new_traj.pose.append(self.pose)
        userdata.trajectory = new_traj
        return 'continue'

    def on_exit(self, userdata):
        pass