#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool
from sonia_common.msg import MultiAddPose
from sonia_common.msg import AddPose, ObstacleArray

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils

class stop_bundle(EventState):

    '''
        Stop the record of a bundle with proc_mapping.

        -- ObstacleID   uint8  The ID of the obstacle you wanted to find.
        -- resetBundle  bool   Indicates if you want to clear a current bundle or not. 

        <= found               Indicates that the proc_mapping found the object you specified.
        <= not_found           Indicates that the proc_mapping didn't find the object you specified.
        <= time_out            Indicates that the proc_mapping didn't find the object you specified within an appropriate delay.
    '''

    def __init__(self, ObstacleID = 1 , resetBundle = False):
        
        super(stop_bundle, self).__init__(outcomes=['found','not_found', 'time_out'])
        
        self.resetBundle = resetBundle
        self.ObstacleArrayUpdated = False
        self.ObstacleArray = None
        self.startBundlePub = rospy.Publisher('/proc_mapping/stop', Bool, queue_size=10)
        self.clearBundlePub = rospy.Publisher('/proc_mapping/clear_bundle', Bool, queue_size=10)
        self.outputPoseSub = rospy.Subscriber('/proc_mapping/obstacle_infos', ObstacleArray, self.pose_cb)
        self.ObstacleID =ObstacleID
    
    def on_enter(self, userdata):
        Logger.log('Sending stop to bundler', Logger.REPORT_HINT)
        self.startBundlePub.publish(False)
        self.time_launch = time()

    def execute(self, userdata):
        time_dif = time() - self.time_launch
        # Time has to experimental values
        if time_dif < 10:
            if(self.ObstacleArray):
                if self.ObstacleArray.obstacles[self.ObstacleID].is_valid:
                    return 'found'
                else:
                    return 'not_found'
        else:
            return 'time_out'

    def on_exit(self, userdata):
        if(self.resetBundle):
            self.clearBundlePub.publish(True)

    def pose_cb(self, data):
        self.poseFound = True
        self.ObstacleArray = data