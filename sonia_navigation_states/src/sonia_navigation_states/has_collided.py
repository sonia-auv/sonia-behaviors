#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
from math import sqrt
import rospy

from flexbe_core import EventState, Logger
#from std_msgs.msg import Bool
#from geometry.msg import Odometry
from nav_msgs.msg import Odometry

class has_collided(EventState):

    '''
        Indicates if the obstacle has been touched
    '''

    def __init__(self, timeout=30, threshold=0.5):

        super(has_collided, self).__init__(outcomes=['target_reached','error'])
        
        self.launch_time = 0
        self.time_diff = 0
        self.param_timeout = timeout
        self.param_threshold = threshold

        self.err_position_x = 0
        self.err_position_y = 0
        self.err_position_z = 0

    def get_traj_err_cb(self, data):
        self.err_position_x = data.pose.pose.position.x
        self.err_position_y = data.pose.pose.position.y
        self.err_position_z = data.pose.pose.position.z

        norm_err = sqrt(self.err_position_x**2 + self.err_position_y**2 + self.err_position_z**2)

        if norm_err > self.param_threshold:
            self.has_collide = True
        
        print("Trajectory error : x = " + str(self.err_position_x) + ", y = " + str(self.err_position_y) + ", z = " + str(self.err_position_z))
        print("Trajectory norm error : " + str(norm_err))

    def on_enter(self, userdata):
        self.is_alive = True
        self.target_reached = False
        self.get_traj_err_sub = rospy.Subscriber('/proc_control/measurment_residual', Odometry, self.get_traj_err_cb)

        # Logger.log('Stopping mouvement', Logger.REPORT_HINT)
        
        # self.reset_trajectory.publish(Bool(True))
        self.launch_time = time()

    def execute(self, userdata):        
        #if self.is_alive == True:
        self.time_diff = time()-self.launch_time
        if self.time_diff > self.param_timeout or self.target_reached == True:
            if self.target_reached == True:
                Logger.log('Collision has been detected', Logger.REPORT_HINT)
                return 'target_reached'
            #else:
            #    Logger.log('Submarine hasnt collided', Logger.REPORT_HINT)
        #else:
        #    Logger.log("Problem with the controller", Logger.REPORT_HINT)
        #    return 'error'

    def on_exit(self, userdata):
        self.get_traj_err_sub.unregister()