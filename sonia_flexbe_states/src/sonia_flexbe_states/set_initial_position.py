#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import sleep, time
from genpy.message import check_type
import rospy

from flexbe_core import EventState, Logger
from rospy.core import logerr
from geometry_msgs.msg import Pose, Point, Quaternion

class set_initial_position(EventState):

    '''
        Set the intial position of the sub.

        <= continue                         Indicates completion of the calculation.

        '''

    def __init__(self):
        
        super(set_initial_position, self).__init__(outcomes=['continue'])

        self.set_initial_position_pub = rospy.Publisher('/initial_condition', Pose, queue_size=2)

    def on_enter(self, userdata):
        pose = Pose()
        pose.position = Point(0.,0.,0.)
        pose.orientation = Quaternion(0.,0.,0.,1)
        self.set_initial_position_pub.publish(pose)

    def execute(self, userdata):
        Logger.log('waiting 5 sec',Logger.REPORT_HINT)
        sleep(5)
        return 'continue'
        # TO DO : add handshake

    def on_exit(self, userdata):
        pass
