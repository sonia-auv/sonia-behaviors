#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy

from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose, Point, Quaternion

class set_initial_position(EventState):

    '''
        Set the intial position of the sub for simulation only

        <= continue                         Indicates completion of the calculation.

        '''

    def __init__(self, simulation=False):
        
        super(set_initial_position, self).__init__(outcomes=['continue'])

        self.set_initial_position_pub = rospy.Publisher('/initial_condition', Pose, queue_size=2)

        self.param_simulation = simulation

    def execute(self, userdata):
        if self.param_simulation == True:
            Logger.log('Setting initial condition', Logger.REPORT_HINT)
            pose = Pose()
            pose.position = Point(0.,0.,0.)
            pose.orientation = Quaternion(0.,0.,0.,1)
            self.set_initial_position_pub.publish(pose)
        else:
            Logger.log('Not in simulation. No need for intial condition', Logger.REPORT_HINT)
        return 'continue'

    def on_exit(self, userdata):
        pass
