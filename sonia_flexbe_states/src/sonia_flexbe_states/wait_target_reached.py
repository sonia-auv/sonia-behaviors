#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
from time import time

from flexbe_core import EventState, Logger
from sonia_common.msg import MpcInfo

class wait_target_reached(EventState):

    '''
        Wait for the trajectory to be completed and the last pose to be reached
        After the trajectory is complete, a wait of 5 seconds is included for the
        target reached.
    '''

    def __init__(self):
        
        super(wait_target_reached, self).__init__(outcomes=['target_reached', 'target_not_reached', 'error'])

        self.get_controller_info_sub = rospy.Subscriber('/proc_control/controller_info', MpcInfo, self.get_controller_info_cb)

    def get_controller_info_cb(self, data):
        self.target_reached = data.target_reached
        self.trajectory_done = data.is_trajectory_done
        self.mpc_status = data.mpc_status

        if self.mpc_status <= 0:
            Logger.log("Problem with the controller", Logger.REPORT_HINT)
            return 'error'

        if self.trajectory_done == True & self.rising_edge == 0:
            self.launch_time = time()
            self.rising_edge = 1
            Logger.log("Trajectory Completed", Logger.REPORT_HINT)

    def on_enter(self, userdata):
        self.target_reached = False
        self.trajectory_done = False
        self.mpc_status = 1
        self.rising_edge = 0
        self.launch_time = 0
        self.time_diff = 0

    def execute(self, userdata):
            if self.trajectory_done == True:
                self.time_diff = time() - self.launch_time
            if self.time_diff > 5:
                if self.target_reached == True:
                    Logger.log("Target Reached", Logger.REPORT_HINT)
                    return 'target_reached'
                else:
                    Logger.log("Target couldn't be reached", Logger.REPORT_HINT)
                    return 'target_not_reached'

    def on_exit(self, userdata):
        self.get_controller_info_sub.unregister()