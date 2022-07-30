#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
from math import sqrt, atan, degrees
import rospy

from flexbe_core import EventState, Logger
from nav_msgs.msg import Odometry
from sonia_common.msg import ObstacleArray

class hydro_heading(EventState):

    '''
        Gives heading detected by hydrophone
    '''

    def __init__(self, timeout=30):

        super(hydro_heading, self).__init__(outcomes=['continue','timeout'],
                                                     output_keys=['heading'])

        self.launch_time = 0
        self.time_diff = 0 

        self.param_timeout = timeout  

        self.x_sub = 0
        self.y_sub = 0
        self.z_sub = 0

        self.x_ping = 0
        self.y_ping = 0
        self.z_ping = 0

        self.heading = 0

        self.got_sub = False
        self.got_ping = False

    def get_states_cb(self, data):
        self.x_sub = data.twist.twist.linear.x
        self.y_sub = data.twist.twist.linear.y
        Logger.log('sub position' + str(self.x_sub) + '-' + str(self.y_sub), Logger.REPORT_HINT)
        self.got_sub = True

    def get_pinger_cb(self, data):
        self.x_ping = data.obstacles[0].pose.position.x
        self.y_ping = data.obstacles[0].pose.position.y
        Logger.log('pinger position' + str(self.x_ping) + '-' +  str(self.y_ping), Logger.REPORT_HINT)
        self.got_ping =  True

    def on_enter(self, userdata):
        self.is_alive = True
        self.get_states_sub = rospy.Subscriber('/proc_nav/auv_states', Odometry, self.get_states_cb)
        self.get_pinger_sub = rospy.Subscriber('/proc_mapping/obstacle_infos', ObstacleArray, self.get_pinger_cb)
        self.launch_time = time()

    def execute(self, userdata):
        self.time_diff = time()-self.launch_time
        if not self.time_diff > self.param_timeout:
            if self.got_sub == True and self.got_ping == True:
                if self.x_sub != self.x_ping:
                    self.heading = degrees(atan((self.y_ping - self.y_sub)/(self.x_ping - self.x_sub)))
                    userdata.heading = self.heading
                    Logger.log('Heading = ' + str(self.heading), Logger.REPORT_HINT)
                    return 'continue'
                else:
                    Logger.log('Heading = ' + str(0), Logger.REPORT_HINT)
                    return 'continue'
        else:
            Logger.log('Timeout', Logger.REPORT_HINT)
            return 'timeout'
            
    def on_exit(self, userdata):
        self.get_states_sub.unregister()
        self.get_pinger_sub.unregister()