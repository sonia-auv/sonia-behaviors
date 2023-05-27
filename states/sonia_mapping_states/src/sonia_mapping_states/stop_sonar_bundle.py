#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool
from sonia_common.msg import ObstacleArray

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils

class stop_sonar_bundle(EventState):

    '''
        Stop the record of a bundle with proc_mapping.

        -- sonarObstacleID   uint8  The sonar obstacle id.
        -- resetSonarBundle  bool   Indicates if you want to clear the current sonar bundle or not. 

        <= found               Indicates that the proc_mapping found the object you specified.
        <= not_found           Indicates that the proc_mapping didn't find the object you specified.
        <= time_out            Indicates that the proc_mapping didn't find the object you specified within an appropriate delay.
    '''

    def __init__(self, sonarObstacleID = 6, resetSonarBundle = False):
        super(stop_sonar_bundle, self).__init__(outcomes=['found', 'not_found', 'time_out'])
        
        # State attributes
        self.resetSonarBundle = resetSonarBundle
        self.ObstacleArrayUpdated = False
        self.ObstacleArray = None
        self.sonarObstacleID = sonarObstacleID
        
        # Publishers
        self.stopSonarBundlePub = rospy.Publisher('/proc_mapping/sonar/stop', Bool, queue_size=1)
        self.clearSonarBundlePub = rospy.Publisher('/proc_mapping/sonar/clear_bundle', Bool, queue_size=1)

        # Subscriber
        self.obstacleArraySub = rospy.Subscriber('/proc_mapping/obstacle_infos', ObstacleArray, self.obstacleCallback)
    
    def on_enter(self, userdata):
        Logger.log('Sending stop to bundler', Logger.REPORT_HINT)
        # Stop the sonar bundle
        self.stopSonarBundlePub.publish(data=True)
        self.time_launch = time()

    def execute(self, userdata):
        time_dif = time() - self.time_launch
        # Time has to experimental values
        if time_dif < 40:
            if self.ObstacleArray:
                if self.ObstacleArray.obstacles[self.sonarObstacleID].is_valid:
                    return 'found'
                else:
                    return 'not_found'
        else:
            return 'time_out'

    def on_exit(self, userdata):
        if self.resetSonarBundle:
            self.clearSonarBundlePub.publish(data=True)

    def obstacleCallback(self, data):
        self.ObstacleArray = data
