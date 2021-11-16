#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import UInt8, Bool
from time import time, sleep

class set_control_mode(EventState):
    """
        Set the control mode of the sub. (change with control modif)
        [...]

        --mode      uint8   The control mode wanted.
        --timeout   uint8   The time allowed to do the change.

        <= continue     Indicate that the mode is different from 0 or has been set.
    """

    def __init__(self, mode, timeout=3):
        super(set_control_mode, self).__init__(outcomes=['continue'])
        self.param_mode = mode
        self.mpc_status = 0
        self.param_timeout = timeout

        self.set_mode = rospy.Publisher('proc_control/set_mode', UInt8, queue_size=2)

    def mpc_status_cb(self, data):
        self.mpc_status = data.data
        Logger.log('MPC status is %s' %str(data.data), Logger.REPORT_HINT)

    def on_enter(self, userdata):
        Logger.log('starting',Logger.REPORT_HINT)
        self.mpc_status_sub = rospy.Subscriber('proc_control/is_mpc_active', Bool, self.mpc_status_cb)
        self.launch_time = time()

    def execute(self, userdata):
        time_dif = time() - self.launch_time
        if time_dif > self.param_timeout:
            Logger.log('ending',Logger.REPORT_HINT)
            if self.mpc_status == True:
                return 'continue'
            else:
                self.set_mode.publish(self.param_mode)
                sleep(self.param_timeout)

    def on_exit(self, userdata):
        self.mpc_status_sub.unregister()