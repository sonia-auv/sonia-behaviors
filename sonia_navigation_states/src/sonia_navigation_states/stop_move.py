#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool
from sonia_common.msg import MpcInfo, MissionTimer
from sonia_navigation_states.src.sonia_navigation_states.modules.navigation_utilities import missionTimerFunc

class stop_move(EventState):

    '''
        Stop an ongoing mouvement (hand break)
    '''

    def __init__(self, timeout=30):

        super(stop_move, self).__init__(outcomes=['target_reached', 'target_not_reached','error'])
        
        self.launch_time = 0
        self.time_diff = 0
        self.param_timeout = timeout
        self.reset_trajectory = rospy.Publisher('/proc_control/reset_trajectory', Bool, queue_size=2)
        self.timeout_pub = rospy.Publisher('/sonia_behaviors/timeout', MissionTimer, queue_size=5)
        self.uniqueID = str(time())

    def get_controller_info_cb(self, data):
        self.target_reached = data.target_reached
        self.is_alive = data.is_mpc_alive

    def on_enter(self, userdata):
        self.is_alive = True
        self.target_reached = False
        self.get_controller_info_sub = rospy.Subscriber('/proc_control/controller_info', MpcInfo, self.get_controller_info_cb)

        Logger.log('Stopping mouvement', Logger.REPORT_HINT)
        
        self.reset_trajectory.publish(Bool(True))
        self.launch_time = time()
        self.timeout_pub.publish(missionTimerFunc("stop_move", self.param_timeout, self.uniqueID, 1))

    def execute(self, userdata):
        if self.is_alive == True:
            self.time_diff = time()-self.launch_time
            if self.time_diff > self.param_timeout or self.target_reached == True:
                if self.target_reached == True:
                    self.timeout_pub.publish(missionTimerFunc("stop_move", self.param_timeout, self.uniqueID, 2))
                    Logger.log('Mouvement has been stopped properly', Logger.REPORT_HINT)
                    return 'target_reached'
                else:
                    self.timeout_pub.publish(missionTimerFunc("stop_move", self.param_timeout, self.uniqueID, 3))
                    Logger.log('Submarine hasnt reached target after stopping', Logger.REPORT_HINT)
                    return 'target_not_reached'
        else:
            self.timeout_pub.publish(missionTimerFunc("stop_move", self.param_timeout, self.uniqueID, 4))
            Logger.log("Problem with the controller", Logger.REPORT_HINT)
            return 'error'

    def on_exit(self, userdata):
        self.get_controller_info_sub.unregister()

