#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import rospy

from flexbe_core import EventState, Logger

class module_check(EventState):

    '''
        State to verify the modules required for a mission

        -- navigation           bool        Verify all the providers/procs for the navigation
        -- vision               bool        Verify all the providers/procs for the vision
        -- mapping              bool        Verify all the provider/proc for the mapping
        -- hydro                bool        Verify all the provider/proc for the hydro
        -- io                   bool        Verify all the provider/proc for the IO board
        -- hardware             bool        Verify all the custom boards

        <= continue                         All the requested modules are functionnal
        <= reboot                           One of the module has been rebooted (TBD)
        <= failed                           One of the module isn't working
    '''

    def __init__(self, navigation=False, vision=False, mapping=False, hydro=False, io=False, hardware=False):
        super(module_check, self).__init__(outcomes=['continue', 'reboot', 'failed'])

        self.navigation = navigation
        self.vision = vision
        self.mapping = mapping
        self.hydro = hydro
        self.io = io
        self.hardware = hardware

        self.message_received = False
        self.error_detected = False
        self.reboot_initiated = False

        # self.get_fault_state = 

    def get_fault_state_cb(self, msg):
        pass

    def on_enter(self, userdata):
        self.message_received = True

    def execute(self, userdata):
        if self.message_received == True:
            if self.error_detected == True:
                Logger.log('One of the selected module in error. Aborting!!!', Logger.REPORT_HINT)
                return 'failed'
            elif self.reboot_initiated == True:
                Logger.log('Error detected, but reboot has been initiated', Logger.REPORT_HINT)
                return 'reboot'
            else:
                Logger.log('No error detected. All module fonctionnal', Logger.REPORT_HINT)
                return 'continue'

    def on_exit(self, userdata):
        pass
            
