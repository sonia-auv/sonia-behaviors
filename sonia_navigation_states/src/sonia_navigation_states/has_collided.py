#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
from math import sqrt
import rospy

from flexbe_core import EventState, Logger
from nav_msgs.msg import Odometry

class has_collided(EventState):

    '''
        Indicates if the obstacle has been touched

        -- timeout      uint8       Time before ending the mission
        -- threshold    float       Value to compare with the error to detect a collision

        <= target_reached           Collision has been detected
        <= error                    Error has been detected
    '''

    def __init__(self, timeout=30, threshold=0.2):

        super(has_collided, self).__init__(outcomes=['target_reached','error'])
        
        self.launch_time = 0
        self.time_diff = 0
        self.err_ori_x = 0
        self.err_ori_y = 0
        self.err_ori_z = 0
        self.param_timeout = timeout
        self.param_threshold = threshold

    def get_traj_err_cb(self, data):
        self.err_ori_x = data.twist.twist.angular.x
        self.err_ori_y = data.twist.twist.angular.y
        self.err_ori_z = data.twist.twist.angular.z
        if abs(self.err_ori_x) > self.param_threshold or abs(self.err_ori_y) > self.param_threshold or abs(self.err_ori_z) > self.param_threshold:
            self.target_reached = True            
        print("Orientation error x : " + str(self.err_ori_x))
        print("Orientation error y : " + str(self.err_ori_y))
        print("Orientation error z : " + str(self.err_ori_z))

    def on_enter(self, userdata):
        self.is_alive = True
        self.target_reached = False
        self.get_traj_err_sub = rospy.Subscriber('/proc_control/measurment_residual', Odometry, self.get_traj_err_cb)
        self.launch_time = time()

    def execute(self, userdata):
        self.time_diff = time()-self.launch_time
        if self.time_diff > self.param_timeout or self.target_reached == True:
            if self.target_reached == True:
                Logger.log('Collision has been detected', Logger.REPORT_HINT)
                return 'target_reached'
            else:
                Logger.log('No collision has been detected', Logger.REPORT_HINT)
                return 'error'

    def on_exit(self, userdata):
        self.get_traj_err_sub.unregister()