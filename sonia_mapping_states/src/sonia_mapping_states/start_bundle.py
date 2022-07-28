#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time

import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Bool, String

class start_bundle(EventState):

    '''
        Start the record of a bundle with proc_mapping.

        -- sonarBundle       bool   Indicates if you want to start the bundle recording for the sonar.
        -- hydroBundle       bool   Indicates if you want to start the bundle recording for the hydro.
        -- sonarTarget       string Name of the object that you want to start bundling.
        -- resetSonarBundle  bool   Indicates if you want to clear the current sonar bundle or not.
        -- resetHydroBundle  bool   Indicates if you want to clear the current hydro bundle or not.

        <= continue                 Indicates that the recording is started.
    '''

    def __init__(self, sonarBundle = True, hydroBundle = False, sonarTarget = 'Buoys', resetSonarBundle = False, resetHydroBundle = False):
        super(start_bundle, self).__init__(outcomes=['continue'])

        # State attributes
        self.sonarBundle = sonarBundle
        self.hydroBundle = hydroBundle
        self.sonarTarget = sonarTarget
        self.resetSonarBundle = resetSonarBundle
        self.resetHydroBundle = resetHydroBundle  

        # Publishers
        self.startSonarBundlePub = rospy.Publisher('/proc_mapping/sonar/start', String, queue_size= 1)
        self.clearSonarBundlePub = rospy.Publisher('/proc_mapping/sonar/clear_bundle', Bool, queue_size= 1)
        self.startHydroBundlePub = rospy.Publisher('/proc_mapping/hydro/start', Bool, queue_size= 1)
        self.clearHydroBundlePub = rospy.Publisher('/proc_mapping/hydro/clear_bundle', Bool, queue_size= 1)
        
    def execute(self, userdata):
        Logger.log('Sending start to bundler', Logger.REPORT_HINT)
        # Clear the sonar bundle if requested.
        if self.resetSonarBundle:
            self.clearSonarBundlePub.publish(data=True) 
        # Clear the hydro bundle if requested.
        if self.resetHydroBundle:
            self.clearHydroBundlePub.publish(data=True)

        # Start the sonar bundle mapping if requested.
        if self.sonarBundle and self.sonarTarget:
            self.startSonarBundlePub.publish(self.sonarTarget)
        # Start the hydro bundle mapping if requested.
        if self.hydroBundle:
            self.startHydroBundlePub.publish(data=True)
        return 'continue'

    def on_exit(self, userdata):
        pass
        