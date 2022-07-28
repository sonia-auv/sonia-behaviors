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

        -- sonarBundle       bool   Indicates if you want to stop the bundle recording for the sonar.
        -- hydroBundle       bool   Indicates if you want to stop the bundle recording for the hydro.
        -- sonarObstacleID   uint8  The ID of the obstacle you wanted to find.
        -- hydroObstacleID   uint8  The hydrophone id.
        -- resetSonarBundle  bool   Indicates if you want to clear the current sonar bundle or not. 
        -- resetHydroBundle  bool   Indicates if you want to clear the current hydro bundle or not. 

        <= found               Indicates that the proc_mapping found the object you specified.
        <= not_found           Indicates that the proc_mapping didn't find the object you specified.
        <= time_out            Indicates that the proc_mapping didn't find the object you specified within an appropriate delay.
    '''

    def __init__(self, sonarObstacleID = 1, hydroObstacleID = 6, sonarBundle = True, hydroBundle = False, resetSonarBundle = False, resetHydroBundle = False):
        super(stop_bundle, self).__init__(outcomes=['found','not_found', 'time_out'])
        
        # State attributes
        self.sonarBundle = sonarBundle
        self.hydroBundle = hydroBundle
        self.resetSonarBundle = resetSonarBundle
        self.resetHydroBundle = resetHydroBundle
        self.ObstacleArrayUpdated = False
        self.ObstacleArray = None
        self.sonarObstacleID = sonarObstacleID
        self.hydroObstacleID = hydroObstacleID

        # Publishers
        self.stopSonarBundlePub = rospy.Publisher('/proc_mapping/sonar/stop', Bool, queue_size=1)
        self.clearSonarBundlePub = rospy.Publisher('/proc_mapping/sonar/clear_bundle', Bool, queue_size=1)
        self.stopHydroBundlePub = rospy.Publisher('/proc_mapping/hydro/stop', Bool, queue_size=1)
        self.clearHydroBundlePub = rospy.Publisher('/proc_mapping/hydro/clear_bundle', Bool, queue_size=1)

        # Subscriber
        self.obstacleArraySub = rospy.Subscriber('/proc_mapping/obstacle_infos', ObstacleArray, self.obstacleCallback)
    
    def on_enter(self, userdata):
        Logger.log('Sending stop to bundler', Logger.REPORT_HINT)
        # Stop the sonar bundle if requested
        if self.sonarBundle:
            self.stopSonarBundlePub.publish(data=True)
        # Stop the sonar bundle if requested
        if self.hydroBundle:
            self.stopHydroBundlePub.publish(data=True)
        self.time_launch = time()

    def execute(self, userdata):
        time_dif = time() - self.time_launch
        # Time has to experimental values
        if time_dif < 10:
            if self.ObstacleArray:
                if self.sonarBundle:
                    if self.ObstacleArray.obstacles[self.sonarObstacleID].is_valid:
                        return 'found'
                    else:
                        return 'not_found'
                if self.hydroBundle:
                    if self.ObstacleArray.obstacles[self.hydroObstacleID].is_valid:
                        return 'found'
                    else:
                        return 'not_found'
        else:
            return 'time_out'

    def on_exit(self, userdata):
        if(self.resetSonarBundle):
            self.clearBundlePub.publish(True)

    def obstacleCallback(self, data):
        self.ObstacleArray = data