#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from rospy.service import ServiceException



from flexbe_core import EventState

class motor_test(EventState):

    '''
        Activate all motors 1 at the time to test them


        <= continue                         Activation of the motors successful
        <= failed                           Failed to activate the motors (Most likely due to kill switch being on)

    '''

    def __init__(self):
        super(motor_test, self).__init__(outcomes=['continue', 'failed'])
        self.dry_test_publisher = rospy.Publisher("/telemetry/dry_run", Bool, queue_size=10, latch=True)
        self.dry_test_service = rospy.ServiceProxy('/provider_thruster/dry_test', Empty)

    def on_enter(self, userdata):
        pass

    def execute(self, userdata):
        try:
            self.dry_test_publisher.publish(data = True)
            self.dry_test_service.call()
            self.dry_test_publisher.publish(data = False)
            return 'continue'
        except ServiceException:
            self.dry_test_publisher.publish(data = False)
            return 'failed'
        except Exception:
            return 'failed'

    def end(self, userdata):
        pass
