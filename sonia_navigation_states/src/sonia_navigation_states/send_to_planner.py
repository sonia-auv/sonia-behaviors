#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

from numpy import int8
import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import MultiAddPose
from std_msgs.msg import Int8

class send_to_planner(EventState):

    '''
        Create the object for the trajectory

        ># input_traj           Trajectory to be compute

        <= continue             Indicates that the trajectory will be compute
        <= failed               Indicates that the waypoints aren't correctly
    '''

    def __init__(self):
        
        super(send_to_planner, self).__init__(outcomes=['continue', 'failed'],
                                              input_keys=['input_traj'])

        self.valid = False
        self.time_launch = 0.0
        self.publish_to_planner = rospy.Publisher('/proc_planner/send_multi_addpose', MultiAddPose, queue_size=1)

    def is_waypoints_valid_cb(self, data):
        self.valid = data.data
    
    def on_enter(self, userdata):
        Logger.log('Sending trajectory to planner', Logger.REPORT_HINT)    

        trajectory = MultiAddPose()
        trajectory.pose = userdata.input_traj.pose
        trajectory.interpolation_method = userdata.input_traj.interpolation_method
        
        self.publish_to_planner.publish(trajectory)
        
        self.time_launch = time()
        self.trajectory_compiled = rospy.Subscriber('/proc_planner/is_waypoints_valid', Int8, self.is_waypoints_valid_cb)

    def execute(self, userdata):
        time_dif = time() - self.time_launch
        # Time has to experimental values
        if time_dif > 5:
            if self.valid == 0:
                return 'continue'
            else:
                Logger.log('Trajectory is invalid. Error code : ' + str(self.valid), Logger.REPORT_HINT)
                return 'failed'

    def on_exit(self, userdata):

        self.trajectory_compiled.unregister()   
        
