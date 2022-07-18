#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import UInt8
from sonia_common.msg import MpcInfo, MissionTimer
from time import time
from sonia_navigation_states.modules.navigation_utilities import missionTimerFunc

class set_control_mode(EventState):
    """
        Set the control mode of the sub. (change with control modif)
        [...]

        -- mode          uint8       The control mode wanted
        -- timeout       uint8       The time allowed to do the change.

        <= continue     Indicate that the mode has been set.
        <= failed       Indicate that the mode hasnt been set
    """

    def __init__(self, mode, timeout=5):
        super(set_control_mode, self).__init__(outcomes=['continue', 'failed'])
        self.param_mode = mode
        self.mpc_mode = 0
        self.param_timeout = timeout
        self.timeout_pub = rospy.Publisher('/sonia_behaviors/timeout', MissionTimer, queue_size=5)
        self.uniqueID = str(time())

        self.set_mode = rospy.Publisher('proc_control/set_mode', UInt8, queue_size=1)

    def mpc_mode_cb(self, data):
        self.mpc_mode = data.mpc_mode

    def on_enter(self, userdata):
        Logger.log('starting',Logger.REPORT_HINT)
        self.mpc_mode_sub = rospy.Subscriber('proc_control/controller_info', MpcInfo, self.mpc_mode_cb)
        self.set_mode.publish(UInt8(self.param_mode))
        self.launch_time = time()
        self.timeout_pub.publish(missionTimerFunc("set_control_mode", self.param_timeout, self.uniqueID, 1))

    def execute(self, userdata):
        time_dif = time() - self.launch_time
        if time_dif > self.param_timeout:
            if self.mpc_mode == self.param_mode:
                self.timeout_pub.publish(missionTimerFunc("set_control_mode", self.param_timeout, self.uniqueID, 2))
                Logger.log('MPC mode has been set :' + str(self.mpc_mode),Logger.REPORT_HINT)
                return 'continue'
            else:
                self.timeout_pub.publish(missionTimerFunc("set_control_mode", self.param_timeout, self.uniqueID, 3))
                Logger.log('MPC mode hasnt been set. Present mode is ' + str(self.mpc_mode),Logger.REPORT_HINT)
                return 'failed'

    def on_exit(self, userdata):
        self.mpc_mode_sub.unregister()