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

        self.launch_time = 0
        self.time_diff = 0
        self.trajectory_done_prev = True
        self.traj_complete = False
        
    def get_controller_info_cb(self, data):
        self.target_reached = data.target_reached
        self.trajectory_done = data.is_trajectory_done
        self.is_alive = data.is_mpc_alive

        Logger.log("Controller => Target Reached :" + str(self.target_reached) + \
            " Trajectory Done :"+ str(self.trajectory_done) + \
            " Previous TD :" + str(self.trajectory_done_prev) + \
            " MPC Alive :" + str(self.is_alive), Logger.REPORT_HINT)

        if self.trajectory_done != self.trajectory_done_prev:
            if self.trajectory_done == False:
                Logger.log("Trajectory has been received", Logger.REPORT_HINT)
            elif self.trajectory_done == True:
                self.launch_time = time()
                self.traj_complete = True
                Logger.log("Trajectory Completed", Logger.REPORT_HINT)

        self.trajectory_done_prev = self.trajectory_done

    def on_enter(self, userdata):
        self.target_reached = False
        self.trajectory_done = True
        self.is_alive = True

        self.get_controller_info_sub = rospy.Subscriber('/proc_control/controller_info', MpcInfo, self.get_controller_info_cb)

    def execute(self, userdata):
        if self.is_alive == True:
            if self.traj_complete == True:
                self.time_diff = time() - self.launch_time
            if self.time_diff > 15 or self.target_reached == True:
                if self.target_reached == True:
                    Logger.log("Target Reached", Logger.REPORT_HINT)
                    return 'target_reached'
                else:
                    Logger.log("Target couldn't be reached", Logger.REPORT_HINT)
                    return 'target_not_reached'
        else:
            Logger.log("Problem with the controller", Logger.REPORT_HINT)
            return 'error'

    def on_exit(self, userdata):
        self.get_controller_info_sub.unregister()