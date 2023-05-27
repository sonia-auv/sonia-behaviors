#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool
from sonia_common.msg import ObstacleArray

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils

class stop_hydro_bundle(EventState):

    '''
        Stop the record of a bundle with proc_mapping.

        -- hydroObstacleID   uint8  The hydrophone id.
        -- resetHydroBundle  bool   Indicates if you want to clear the current hydro bundle or not. 

        <= found               Indicates that the proc_mapping found the object you specified.
        <= not_found           Indicates that the proc_mapping didn't find the object you specified.
        <= time_out            Indicates that the proc_mapping didn't find the object you specified within an appropriate delay.
    '''

    def __init__(self, hydroObstacleID = 5, resetHydroBundle = False):
        super(stop_hydro_bundle, self).__init__(outcomes=['found', 'not_found', 'time_out'])
        
        # State attributes
        self.resetHydroBundle = resetHydroBundle
        self.ObstacleArrayUpdated = False
        self.ObstacleArray = None
        self.hydroObstacleID = hydroObstacleID
        
        # Publishers
        self.stopHydroBundlePub = rospy.Publisher('/proc_mapping/hydro/stop', Bool, queue_size=1)
        self.clearHydroBundlePub = rospy.Publisher('/proc_mapping/hydro/clear_bundle', Bool, queue_size=1)

        # Subscriber
        self.obstacleArraySub = rospy.Subscriber('/proc_mapping/obstacle_infos', ObstacleArray, self.obstacleCallback)
    
    def on_enter(self, userdata):
        Logger.log('Sending stop to bundler', Logger.REPORT_HINT)
        # Stop the hydro bundle
        self.stopHydroBundlePub.publish(data=True)
        self.time_launch = time()

    def execute(self, userdata):
        time_dif = time() - self.time_launch
        # Time has to experimental values
        if time_dif < 10:
            if self.ObstacleArray:
                if self.ObstacleArray.obstacles[self.hydroObstacleID].is_valid:
                    return 'found'
                else:
                    return 'not_found'
        else:
            return 'time_out'

    def on_exit(self, userdata):
        if self.resetHydroBundle:
            self.clearHydroBundlePub.publish(data=True)

    def obstacleCallback(self, data):
        self.ObstacleArray = data