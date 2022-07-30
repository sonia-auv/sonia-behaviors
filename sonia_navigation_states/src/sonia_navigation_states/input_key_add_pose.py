#!/usr/bin/env python 
#-*- coding: utf-8 -*-

# standars includes
from time import time
import rospy

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils

from flexbe_core import EventState, Logger
from sonia_common.msg import MultiAddPose

class input_key_add_pose(EventState):

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
                                            4 : Absolute depth and everything else relative
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

    def __init__(self, orientationX=0.0, orientationY=0.0, frame=1, speed=0, precision=0, long_rotation=False):
        
        super(input_key_add_pose, self).__init__(outcomes=['continue'],
                                                     input_keys=['input_traj', 'pose_x', 'pose_y', 'pose_z', 'angle'],
                                                     output_keys=['trajectory'])
        # create addpose object
        self.param_orientation_x = orientationX
        self.param_orientation_y = orientationY
        self.param_frame = frame
        self.param_speed = speed
        self.param_precision = precision
        self.param_long_rotation = long_rotation


    def on_enter(self, userdata):
        self.pose = navUtils.addpose(userdata.pose_x, userdata.pose_y, userdata.pose_z, self.param_orientation_x, self.param_orientation_y, -userdata.angle, self.param_frame, self.param_speed, self.param_precision, self.param_long_rotation)


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