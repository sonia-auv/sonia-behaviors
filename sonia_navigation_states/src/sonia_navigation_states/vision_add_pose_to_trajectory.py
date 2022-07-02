#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from flexbe_core import EventState, Logger
from sonia_common.msg import MultiAddPose

class vision_add_pose_to_trajectory(EventState):

    '''
        Create the pose for the trajectory with a message Addpose.
        Should only be use for the vision state.

        ># input_traj       MultiAddPose    Input trajectory
        ># pose             AddPose         Input pose

        #> trajectory       MultiAddPose    Output trajectory with the new pose added

        <= continue                         Indicates completion.
        '''

    def __init__(self):
        
        super(vision_add_pose_to_trajectory, self).__init__(outcomes=['continue'],
                                                     input_keys=['input_traj', 'pose'],
                                                     output_keys=['trajectory'])

    def execute(self, userdata):
        traj = userdata.input_traj
        pose = userdata.pose
        new_traj = MultiAddPose()
        if not traj.pose:
            Logger.log('First position of the trajectory', Logger.REPORT_HINT)
        else:
            Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)            
            new_traj.pose = list(traj.pose)

        Logger.log('Pose : x = '+ str(pose.position.x) + ', y = ' + str(pose.position.y) + ' z = ' + str(pose.position.z) + \
            ' roll = ' + str(pose.orientation.x) + ' pitch = ' + str(pose.orientation.y) + ' yaw = ' + str(pose.orientation.z) + \
            ' frame = ' + str(pose.frame), Logger.REPORT_HINT)

        new_traj.pose.append(pose)
        new_traj.interpolation_method = traj.interpolation_method
        userdata.trajectory = new_traj
        return 'continue'

    def on_exit(self, userdata):
        pass