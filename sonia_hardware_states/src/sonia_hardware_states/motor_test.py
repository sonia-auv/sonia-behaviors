#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import time
import rospy
import threading
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from flexbe_core import EventState, Logger
from sonia_common.msg import MissionTimer
from sonia_navigation_states.modules.navigation_utilities import missionTimerFunc

class motor_test(EventState):

    '''
        Activate all motors 1 at the time to test them


        <= continue                         Activation of the motors successful
        <= failed                           Failed to activate the motors (Most likely due to kill switch being on)
        <= timed out                        Activation timed out

    '''

    def __init__(self,	 timeout=20):
        super(motor_test, self).__init__(outcomes=['continue', 'failed', 'timeout'])
        
        self.time_start = None
        self.timeout = timeout
        self.response = ''

        self.timeout_pub = rospy.Publisher('/sonia_behaviors/timeout', MissionTimer, queue_size=5)
        
        
        self.dry_test_publisher = rospy.Publisher("/telemetry/dry_run", Bool, queue_size=10, latch=True)
        self.dry_test_service = rospy.ServiceProxy('/provider_thruster/dry_test', Empty)

    def on_enter(self, userdata):
        self.time_start = time()
        self.timeout_pub.publish(missionTimerFunc(f"motor_test", self.timeout, str(self.time_start), 1))
        Logger.log(f'Motor_test is launched', Logger.REPORT_HINT)
        
        self.dry_test_publisher.publish(data = True)
        newThread = Threads(self.dry_test_service)
        newThread.start()

    def reply_cb(self, data):
        if data.element == self.element and self.side == data.side:
            pass
            '''if data.response == ActuatorSendReply.RESPONSE_FAILURE:
                self.timeout_pub.publish(missionTimerFunc("motor_test", self.timeout, str(self.time_start), 4))
                self.response = 'failed'
            elif data.response == ActuatorSendReply.RESPONSE_SUCCESS:
                self.timeout_pub.publish(missionTimerFunc("motor_test", self.timeout, str(self.time_start), 2))
                self.response = 'continue'
            elif data.response == ActuatorSendReply.RESPONSE_TIMED_OUT:
                self.timeout_pub.publish(missionTimerFunc("motor_test", self.timeout, str(self.time_start), 3))
                self.response = 'timeout'
            else:
                Logger.log(f"/provider_actuators/do_action_from_actuators sent a message with the response code {data.response} which should not exist", Logger.REPORT_ERROR)'''
                
        self.dry_test_publisher.publish(data = False)

    def execute(self, userdata):
        if self.response != "":
            return self.response
        if time() - self.time_start >= self.timeout:
            self.timeout_pub.publish(missionTimerFunc("motor_test", self.timeout, str(self.time_start), 3))
            Logger.log('Timeout Reached', Logger.REPORT_HINT)
            return 'timeout'

    def end(self, userdata):
        self.action_sub.unregister()


class Threads(threading.Thread):
    def __init__(self, dryTestService):
        super(Threads, self).__init__()
        self.dryTestService = dryTestService
    
    def run(self):
        try:
            self.dryTestService.call()
        except rospy.ServiceException as e:
            print(e)
            rospy.logerr('Provider Thruster Node is not started')