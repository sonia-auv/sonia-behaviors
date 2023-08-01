#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

from numpy import int8
import rospy
import math

from flexbe_core import EventState, Logger
import sonia_navigation_states.modules.navigation_utilities as navUtils
from sonia_common.msg import MultiAddPose
from std_msgs.msg import Int8

class rotate_around_target(EventState):

    '''
        Create the object for the trajectory

        ># input_traj           Trajectory to be compute

        <= continue             Indicates that the trajectory will be compute
        <= failed               Indicates that the waypoints aren't correctly
    '''

    def __init__(self, angle, distance=1):
        
        super(rotate_around_target, self).__init__(outcomes=['continue', 'failed'],  input_keys=["calc_block"])

        self.valid = False
        self.time_launch = 0.0
        self.distance = distance
        self.angle = angle
        self.publish_to_planner = rospy.Publisher('/proc_planner/send_multi_addpose', MultiAddPose, queue_size=1)

    def is_waypoints_valid_cb(self, data):
        self.valid = data.data
    
    def on_enter(self, userdata):
        Logger.log('Sending trajectory to planner', Logger.REPORT_HINT)    

        if self.angle > 0 :
            angle = self.angle
        else:
            angle = userdata.calc_block[abs(self.angle)]
        
        radians = math.radians(angle)
        positionY = self.distance * math.sin(radians)
        positionX = self.distance - (self.distance * math.cos(radians))

        trajectory = MultiAddPose()
        trajectory.pose.append(navUtils.addpose(positionX, positionY, 0, 0, 0, -angle, 1, 0, 0, False))
        
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
        
