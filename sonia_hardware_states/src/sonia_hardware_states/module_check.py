#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger
from sonia_common.msg import FaultDetection

class module_check(EventState):

    '''
        State to verify the modules required for a mission

        -- navigation           bool        Verify all the providers/procs for the navigation
        -- vision               bool        Verify all the providers/procs for the vision
        -- mapping              bool        Verify all the provider/proc for the mapping
        -- hydro                bool        Verify all the provider/proc for the hydro
        -- io                   bool        Verify all the provider/proc for the IO board
        -- underwater_com       bool        Verify all the provider/proc for the underwater communication
        -- power                bool        Verify all the providers/proc related to the power management
        -- internal_com         bool        Verify the rs485 communication

        <= continue                         All the requested modules are functionnal
        <= reboot                           One of the module has been rebooted (TBD)
        <= failed                           One of the module isn't working
    '''

    def __init__(self, navigation=True, vision=False, mapping=False, hydro=False, io=False, underwater_com=False, power=True, internal_com=True):
        super(module_check, self).__init__(outcomes=['continue', 'reboot', 'failed'])

        self.modules = tuple((navigation, vision, mapping, hydro, io, underwater_com, power, internal_com))
        self.message_received = False
        self.error_detected = False
        self.reboot_initiated = False

    def get_fault_state_cb(self, msg):
        #The rosmessage has been declared like this for easier debugging for the team. More manipulation needs to be done here.
        msg_error = tuple((msg.navigation, msg.vision, msg.mapping, msg.hydro, msg.io, msg.underwater_com, msg.power, msg.internal_com))
        dictionnary_module = tuple((("navigation", "vision", "mapping", "hydro", "io", "underwater_com", "power", "internal_com")))
        
        for x in range(0, len(self.modules)):
            if msg_error[x] == False and self.modules[x] == True:
                Logger.log('Error with the module ' + dictionnary_module[x], Logger.REPORT_HINT)
                self.error_detected = True

        self.message_received = True

    def on_enter(self, userdata):
        self.get_fault_state = rospy.Subscriber('/proc_fault/fault_detection', FaultDetection, self.get_fault_state_cb)

    def execute(self, userdata):
        if self.message_received == True:
            if self.error_detected == True:
                Logger.log('One of the selected module in error. Aborting!!!', Logger.REPORT_HINT)
                return 'failed'
            elif self.reboot_initiated == True:
                Logger.log('Error detected, but reboot has been initiated', Logger.REPORT_HINT)
                return 'reboot'
            else:
                Logger.log('No error detected. All module functional for this mission', Logger.REPORT_HINT)
                return 'continue'

    def on_exit(self, userdata):
        self.get_fault_state.unregister()
            
