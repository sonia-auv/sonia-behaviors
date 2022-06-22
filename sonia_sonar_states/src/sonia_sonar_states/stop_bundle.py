#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool
from sonia_common.msg import MultiAddPose
from sonia_common.msg import AddPose

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils

class stop_bundle(EventState):

    '''
        Create the object for the trajectory

        <= continue             Indicates that the trajectory will be compute
        <= failed               Indicates that the waypoints aren't correctly
    '''

    def __init__(self, resetBundle = False, frame=1, speed=0, precision=0, long_rotation=False):
        
        super(stop_bundle, self).__init__(outcomes=['continue', 'time_out'], output_keys=['trajectory'])
        self.time_launch = time()
        self.resetBundle = resetBundle
        self.positionX = 0.0
        self.positionY = 0.0
        self.positionZ = 0.0
        self.orientationX = 0.0
        self.orientationY = 0.0
        self.orientationZ = 0.0
        self.frame = frame
        self.speed = speed
        self.precision = precision
        self.long_rotation = long_rotation
        self.poseFound = False
        self.pose = None
        self.startBundlePub = rospy.Publisher('/proc_mapping/start_stop', Bool, queue_size=10)
        self.clearBundlePub = rospy.Publisher('/proc_mapping/clear_bundle', Bool, queue_size=10)
        self.outputPoseSub = rospy.Subscriber('proc_mapping/output_pose', AddPose, self.pose_cb)

    
    def on_enter(self, userdata):
        Logger.log('Sending start to bundler', Logger.REPORT_HINT)
        self.startBundlePub.publish(False)

    def execute(self, userdata):
        time_dif = time() - self.time_launch
        # Time has to experimental values
        if time_dif < 60:
            if(self.poseFound):
                userdata.trajectory = navUtils.addpose(self.pose.position.x, self.pose.position.Y, self.pose.position.Z, self.pose.orientation.X,
                                                      self.pose.orientation.Y, self.pose.orientation.Z,self.frame, self.speed, self.precision, self.long_rotation)
        else:
            return 'time_out'

    def on_exit(self, userdata):
        if(self.resetBundle):
            self.clearBundlePub.publish(True)

    def pose_cb(self, data):
        self.poseFound = True
        self.pose = data