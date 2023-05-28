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

class create_pose(EventState):

    '''
        (!!!! DEPRECATED !!!!!) Move the submarine by defining every parameter.
        
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

        <= continue                     Indicates that the pose has been created

        '''

    def __init__(self, positionX, positionY, positionZ, orientationX, orientationY, orientationZ, frame, time=5, precision=0, rotation=True):
        
        super(create_pose, self).__init__(outcomes=['continue'],
                                          output_keys=['pose'])

        self.pose = AddPose()
        self.pose.position = Point(positionX,positionY,positionZ)
        self.pose.orientation = Vector3(orientationX,orientationY,orientationZ)
        self.pose.frame = frame
        self.pose.speed = time
        self.pose.fine = precision
        self.pose.rotation = rotation

    def on_enter(self, userdata):
        pass
    def execute(self, userdata):
        userdata.pose = self.pose
        return 'continue'
    def on_exit(self, userdata):
        pass