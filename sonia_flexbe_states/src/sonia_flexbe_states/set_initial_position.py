#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import sleep, time
from genpy.message import check_type
import rospy

from flexbe_core import EventState, Logger
from rospy.core import logerr
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool

class set_initial_position(EventState):

    '''
        Set the intial position of the sub for simulation only

        <= continue                         Indicates completion of the calculation.

        '''

    def __init__(self, simulation):
        
        super(set_initial_position, self).__init__(outcomes=['continue', 'wait'])

        self.set_initial_position_pub = rospy.Publisher('/initial_condition', Pose, queue_size=2)
        
        self.target_reach = False
        self.param_simulation = simulation

    def target_reach_cb(self, data):
        self.target_reach = data.data

    def on_enter(self, userdata):
        pose = Pose()
        pose.position = Point(0.,0.,0.)
        pose.orientation = Quaternion(0.,0.,0.,1)
        self.set_initial_position_pub.publish(pose)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', Bool, self.target_reach_cb)

    def execute(self, userdata):
        if self.param_simulation == True :
            if self.target_reach == True :
                return 'continue'
        else :
            return 'wait'

    def on_exit(self, userdata):
        pass
