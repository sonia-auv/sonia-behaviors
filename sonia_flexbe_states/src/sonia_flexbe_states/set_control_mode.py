#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import UInt8, Int8
from time import time

class set_control_mode(EventState):
    """
        Set the control mode of the sub.
        [...]

        --mode      uint8   The control mode wanted.
        --timeout   uint8   The time allowed to do the change.

        <= continue     Indicate that the mode is different from 0.
        <= failed       Indicate that the mode didn't changed.
    """

    def __init__(self, mode, timeout=3):
        super(set_control_mode, self).__init__(outcomes=['continue', 'failed'])
        self.param_mode = mode
        self.mpc_status = 0
        self.param_timeout = timeout

        self.set_mode = rospy.Publisher('proc_control/set_mode', UInt8, queue_size=2)

    def mpc_status_cb(self, data):
        self.mpc_status = data 

    def on_enter(self, userdata):
        Logger.log('starting',Logger.REPORT_HINT)
        self.set_mode.publish(self.param_mode)
        self.mpc_status_sub = rospy.Subscriber('proc_control/mpc_status', Int8, self.mpc_status_cb)
        self.launch_time = time()

    def execute(self, userdata):
        time_dif = time() - self.launch_time
        if time_dif > self.param_timeout:
            Logger.log('ending',Logger.REPORT_HINT)
            if self.mpc_status != 0:
                return 'continue'
            else:
                return 'failed'

    def on_exit(self, userdata):
        pass    

    
