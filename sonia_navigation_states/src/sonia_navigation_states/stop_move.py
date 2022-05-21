#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool
from sonia_common.msg import MpcInfo

class stop_move(EventState):

    '''
        Stop an ongoing mouvement (hand break)
    '''

    def __init__(self, timeout=3):

        super(stop_move, self).__init__(outcomes=['continue', 'failed', 'error'])

        self.launch_time = 0
        self.time_diff = 0
        self.trajectory_done_prev = True
        self.traj_complete = False
        self.param_timeout = timeout
        self.reset_trajectory = rospy.Publisher('/proc_control/reset_trajectory', Bool, queue_size=2)
        self.is_alive = False


    def get_controller_info_cb(self, data):
        self.target_reached = data.target_reached
        self.trajectory_done = data.is_trajectory_done
        self.is_alive = data.is_mpc_alive

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
        self.start_time = time()
        self.get_controller_info_sub = rospy.Subscriber('/proc_control/controller_info', MpcInfo, self.get_controller_info_cb)
        
        Logger.log('Stopping mouvement', Logger.REPORT_HINT)
        self.reset_trajectory.publish(Bool(True))

    def execute(self, userdata):
        if self.is_alive == True:
            if self.traj_complete == True:
                self.time_diff = time() - self.launch_time
            if self.time_diff > self.param_timeout or self.target_reached == True:
                if self.target_reached == True:
                    Logger.log("Mouvement has been stopped properly", Logger.REPORT_HINT)
                    return 'continue'
                else:
                    Logger.log("Submarine hasnt reached target after stopping", Logger.REPORT_HINT)
                    return 'failed'
        else:
            Logger.log("Problem with the controller", Logger.REPORT_HINT)
            return 'error'


    def on_exit(self, userdata):
        self.get_controller_info_sub.unregister()
