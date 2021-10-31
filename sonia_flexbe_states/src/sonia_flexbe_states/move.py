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

class move(EventState):

    '''
        Move the submarine by defining every parameter.
        [...]
        
        -- positionX        uint8       The function that performs [...]
        -- positionY        uint8       The function that performs [...]
        -- positionZ        uint8       The function that performs [...]
        -- orientationX     uint8       The function that performs [...]
        -- orientationY     uint8       The function that performs [...]
        -- orientationZ     uint8       The function that performs [...]
        -- frame            uint8       0 : Absolute position and absolute angle
                                            1 : Relative position and relative angle
                                            2 : Relative position and absolute angle
                                            3 : Absolute position and relative angle 
        -- time             uint8       The function that performs [...]
        -- precision        float64     The function that performs [...]
        -- path             bool        The function that performs [...]

        ># positionX        uint8       Input to the calculation function.

        #> output_value     object      The result of the calculation.

        <= done                         Indicates completion of the calculation.

        '''

    def __init__(self, positionX, positionY, positionZ, orientationX, orientationY, orientationZ, frame, time=5, precision=0, rotation=True):
        
        super(move, self).__init__(outcomes=['continue', 'failed'])

        self.target_reached = False

        self.param_distance_x = positionX
        self.param_distance_y = positionY
        self.param_distance_z = positionZ
        self.param_orientation_x = orientationX
        self.param_orientation_y = orientationY
        self.param_orientation_z = orientationZ
        self.param_frame = frame
        self.param_time = time
        self.param_precision = precision
        self.param_rotation = rotation
        self.actual_x = 0
        self.actual_y = 0
        self.actual_z = 0

        self.add_pose = rospy.Publisher('/proc_control/add_pose', AddPose, queue_size=2)
        self.get_current_position_sub = rospy.Subscriber('/telemetry/auv_states', Odometry, self.get_current_position_cb)

    def target_reach_cb(self, data):
        self.target_reached = data

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

    def on_enter(self, userdata):

        self.check_frame()
        self.check_speed()

        self.add_pose.publish(Point(self.param_distance_x,self.param_distance_y, self.param_distance_z),
            Vector3(self.param_orientation_x, self.param_orientation_y, self.param_orientation_z),
            self.param_frame, self.param_time, self.param_precision, self.param_rotation )
        self.time_launch = time()

        rospy.loginfo('Set position x = %f' % self.param_distance_x)
        rospy.loginfo('Set position y = %f' % self.param_distance_y)
        rospy.loginfo('Set position z = %f' % self.param_distance_z)
        rospy.loginfo('Set orientation x = %f' % self.param_orientation_x)
        rospy.loginfo('Set orientation y = %f' % self.param_orientation_y)
        rospy.loginfo('Set orientation z = %f' % self.param_orientation_z)
        rospy.loginfo('Set frame = %f' % self.param_frame)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', Bool, self.target_reach_cb)

    def execute(self, userdata):
        Logger.log('starting',Logger.REPORT_HINT)
        time_dif = time() - self.time_launch
        if time_dif > self.param_time:
            Logger.log('ending',Logger.REPORT_HINT)
            if self.target_reached == True:
                return 'continue'
            else:
                return 'failed'        


    def on_exit(self, userdata):
        self.target_reach_sub.unregister()
        self.get_current_position_sub.unregister()
