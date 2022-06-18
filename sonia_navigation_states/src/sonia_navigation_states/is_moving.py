#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy
import math

from time import time
from flexbe_core import EventState, Logger
from nav_msgs.msg import Odometry
from sonia_common.msg import MpcInfo
from tf.transformations import euler_from_quaternion

class is_moving(EventState):

    '''
        Stop an ongoing mouvement (hand break)

        -- timeout      uint8       Time before stopping the state
        -- tolerance    float       Sum of all tolerances to state the sub is moving or stopped

        <= stopped                  The sub is stopped
        <= moving                   The sub is moving
        <= error                    Problem with the controller
    '''

    def __init__(self, timeout=30, tolerance=0.1):

        super(is_moving, self).__init__(outcomes=['stopped', 'moving','error'])
        
        self.launch_time = 0
        self.time_diff = 0
        self.prev_time = 0
        self.param_timeout = timeout
        self.param_tolerance = tolerance

        self.prev_position_x = 0
        self.prev_position_y = 0
        self.prev_position_z = 0
        self.prev_orientation_x = 0
        self.prev_orientation_y = 0
        self.prev_orientation_z = 0

    def on_enter(self, userdata):
        self.is_alive = True
        self.stopped = False
        self.target_reached = False
        self.first = True
        self.get_controller_info_sub = rospy.Subscriber('/proc_control/controller_info', MpcInfo, self.get_controller_info_cb)
        self.get_current_states_sub = rospy.Subscriber('/proc_nav/auv_states', Odometry, self.get_current_states_cb)

        Logger.log('Checking is the sub is moving', Logger.REPORT_HINT)
        
        self.launch_time = time()

    def get_controller_info_cb(self, data):
        self.target_reached = data.target_reached
        self.is_alive = data.is_mpc_alive

    def get_current_states_cb(self, data):
        if self.first:
            self.prev_position_x = data.pose.pose.position.x
            self.prev_position_y = data.pose.pose.position.y
            self.prev_position_z = data.pose.pose.position.z
            self.prev_orientation_x = math.degrees(euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w],'szyx')[2])
            self.prev_orientation_y = math.degrees(euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w],'szyx')[1])
            self.prev_orientation_z = math.degrees(euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w],'szyx')[0])

        if self.time_diff - self.prev_time >= 2:
            self.prev_time = self.time_diff

            self.diff_position_x = self.prev_position_x - data.pose.pose.position.x
            self.diff_position_y = self.prev_position_y - data.pose.pose.position.y
            self.diff_position_z = self.prev_position_z - data.pose.pose.position.z
            self.diff_orientation_x = self.prev_orientation_x - math.degrees(euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w],'szyx')[2])
            self.diff_orientation_y = self.prev_orientation_y - math.degrees(euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w],'szyx')[1])
            self.diff_orientation_z = self.prev_orientation_z - math.degrees(euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w],'szyx')[0])
               
            if sum([abs(self.diff_position_x),abs(self.diff_position_y),abs(self.diff_position_z),abs(self.diff_orientation_x),abs(self.diff_orientation_y),abs(self.diff_orientation_z)]) < self.param_tolerance:
                print('stopped')
                self.stopped = True
            else:
                print('moving')
                self.prev_position_x=data.pose.pose.position.x
                self.prev_position_y=data.pose.pose.position.y
                self.prev_position_z=data.pose.pose.position.z
                self.prev_orientation_x=math.degrees(euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w],'szyx')[2])
                self.prev_orientation_y=math.degrees(euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w],'szyx')[1])
                self.prev_orientation_z=math.degrees(euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w],'szyx')[0])

    def execute(self, userdata):
        if self.is_alive:
            self.time_diff = time()-self.launch_time
            if self.time_diff > self.param_timeout or self.target_reached or self.stopped:
                if self.stopped:
                    Logger.log('The sub is not moving, bypassing Target Reached', Logger.REPORT_HINT)
                    return 'stopped'
                elif self.target_reached:
                    Logger.log('Finally got target_reached')
                    return 'stopped'
                else:
                    Logger.log('The sub is still moving', Logger.REPORT_HINT)
                    return 'moving'
        else:
            Logger.log("Problem with the controller", Logger.REPORT_HINT)
            return 'error'

    def on_exit(self, userdata):
        self.get_controller_info_sub.unregister()
        self.get_current_states_sub.unregister()

