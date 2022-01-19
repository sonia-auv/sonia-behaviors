#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import sleep, time
from genpy.message import check_type
import rospy

from flexbe_core import EventState, Logger
from rospy.core import logerr
from sonia_common.msg import AddPose
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry

class move_to_target(EventState):

    '''
        Move the submarine by defining every parameter.

        <= continue                         Indicates that the sub has moved to the target.
        <= failed                           Couldn't move to the desired pose

        '''

    def __init__(self):
        
        super(move_to_target, self).__init__(outcomes=['continue', 'failed'],
                                             input_keys=['pose'])

        self.target_reached = False
        self.actual_x = 0
        self.actual_y = 0
        self.actual_z = 0

        self.add_pose = rospy.Publisher('/proc_control/add_pose', AddPose, queue_size=2)
        self.get_current_position_sub = rospy.Subscriber('/telemetry/auv_states', Odometry, self.get_current_position_cb)

    def target_reach_cb(self, data):
        self.target_reached = data.data

    def check_frame(self):
        if self.param_frame > 3:
            rospy.logerr('Wrong frame selection, must be between 0 and 3 included')
            return 'failed'

    def get_current_position_cb(self, data):
        self.actual_x = data.pose.pose.position.x
        self.actual_y = data.pose.pose.position.y
        self.actual_z = data.pose.pose.position.z

    def check_speed(self):
        if self.param_frame == 1 or self.param_frame == 2:
            speedx = abs(self.param_distance_x) / self.param_time
            speedy = abs(self.param_distance_y) / self.param_time
            speedz = abs(self.param_distance_z) / self.param_time
        else:
            speedx = abs(self.actual_x - self.param_distance_x) / self.param_time
            speedy = abs(self.actual_y - self.param_distance_y) / self.param_time
            speedz = abs(self.actual_z - self.param_distance_z) / self.param_time

        if speedz > 1 or speedy > 1 or speedx > 1: 
            self.param_time = max(self.param_distance_x,self.param_distance_y,self.param_distance_z) / 0.8
            Logger.log('Changing time for %s' %self.param_time,Logger.REPORT_HINT)

    def on_enter(self, userdata):
        Logger.log('starting',Logger.REPORT_HINT)
        
        self.param_distance_x = userdata.pose.position.x
        self.param_distance_y = userdata.pose.position.y
        self.param_distance_z = userdata.pose.position.z
        self.param_orientation_x = userdata.pose.orientation.x
        self.param_orientation_y = userdata.pose.orientation.y
        self.param_orientation_z = userdata.pose.orientation.z
        self.param_frame = userdata.pose.frame
        self.param_time = userdata.pose.speed
        self.param_precision = userdata.pose.fine
        self.param_rotation = userdata.pose.rotation
        
        self.check_frame()
        self.check_speed()

        self.add_pose.publish(Point(self.param_distance_x,self.param_distance_y, self.param_distance_z),
            Vector3(self.param_orientation_x, self.param_orientation_y, self.param_orientation_z),
            self.param_frame, self.param_time, self.param_precision, self.param_rotation )
        self.time_launch = time()

        Logger.log('Pose : x = '+ str(self.param_distance_x) + ', y = ' + str(self.param_distance_y) + ' z = ' + str(self.param_distance_z) + \
            ' roll = ' + str(self.param_orientation_x) + ' pitch = ' + str(self.param_orientation_y) + ' yaw = ' + str(self.param_orientation_z) + \
            ' frame = ' + str(self.param_frame), Logger.REPORT_HINT)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', Bool, self.target_reach_cb)

    def execute(self, userdata):
        time_dif = time() - self.time_launch
        if time_dif > self.param_time:
            if self.target_reached == True:
                Logger.log('Target reached', Logger.REPORT_HINT)
                return 'continue'
            elif time_dif > self.param_time + 30:
                Logger.log('Target not reached', Logger.REPORT_HINT)
                return 'failed'      

    def on_exit(self, userdata):
        self.target_reach_sub.unregister()
        self.get_current_position_sub.unregister()
