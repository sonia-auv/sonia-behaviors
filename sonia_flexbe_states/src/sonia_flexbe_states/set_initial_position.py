#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Int8

class set_initial_position(EventState):

    '''
        Set the intial position of the sub for simulation only

        <= continue                         Indicates completion of the calculation.

        '''

    def __init__(self, simulation=False, timeout=10):
        
        super(set_initial_position, self).__init__(outcomes=['continue'])

        self.set_initial_position_pub = rospy.Publisher('/initial_condition', Pose, queue_size=2)

        self.param_simulation = simulation
        self.param_timeout = timeout

    def on_enter(self, userdata):
        self.start_time = time()

    def execute(self, userdata):
        actual = time()-self.start_time
        if actual > self.param_timeout:
            if self.param_simulation == True:
                Logger.log('Setting initial condition', Logger.REPORT_HINT)
                pose = Pose()
                pose.position = Point(0.,0.,0.)
                pose.orientation = Quaternion(0.,0.,0.,1)
                self.set_initial_position_pub.publish(pose)
            return 'continue'

    def on_exit(self, userdata):
        pass
