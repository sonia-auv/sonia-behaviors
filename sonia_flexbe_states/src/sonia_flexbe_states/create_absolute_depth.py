#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import AddPose
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry

class create_absolute_depth(EventState):

    '''
        Move the submarine by defining every parameter.

        -- positionZ        uint8       The absolute depth desired to reach

        <= continue                     Indicates that the pose has been created

        '''

    def __init__(self, positionZ):
        
        super(create_absolute_depth, self).__init__(outcomes=['continue'],
                                          output_keys=['pose'])

        self.param_positionZ = positionZ

        self.get_current_position_sub = rospy.Subscriber('/telemetry/auv_states', Odometry, self.get_current_position_cb)

    def get_current_position_cb(self, data):
        self.actual_z = data.pose.pose.position.z

    def on_enter(self, userdata):
        self.pose = AddPose()
        self.pose.position = Point(0.,0.,0.)
        self.pose.orientation = Vector3(0.,0.,0.)
        self.pose.frame = 1
        self.pose.speed = 5
        self.pose.fine = 0.
        self.pose.rotation = True
        
        self.actual_z = 0
        
    def execute(self, userdata):
        if self.actual_z != 0 :
            self.pose.position.z = self.param_positionZ - self.actual_z
            Logger.log('Mouvement in Z with be of %f m' %self.pose.position.z, Logger.REPORT_HINT)
            userdata.pose = self.pose
            return 'continue'

    def on_exit(self, userdata):
        pass