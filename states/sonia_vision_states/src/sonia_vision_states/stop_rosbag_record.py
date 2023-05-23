#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import subprocess, psutil

from flexbe_core import EventState, Logger

class stop_rosbag_record(EventState):
    '''
        Stop the rosbag entered
        [...]

        ># rosbag_proc                      Record process to kill after bag is done
        ># command                          Command used to start the bag

        <= continue			Indicates that the record has started
        <= failed			Indicates that the record didn't start

    '''
    def __init__(self):
        
        super(stop_rosbag_record, self).__init__(outcomes = ['continue', 'failed'],
                                                  input_keys = ['rosbag_proc','command'])
        

    def execute(self, userdata):
        try:
            for proc in psutil.process_iter():
                if "record" in proc.name() and set(userdata.command[2:]).issubset(proc.cmdline()):
                    proc.send_signal(subprocess.signal.SIGINT)

            userdata.rosbag_proc.send_signal(subprocess.signal.SIGINT)
        except:
            return 'failed'
        return 'continue'

    def on_exit(self, userdata):
        Logger.log('Rosbag stopped', Logger.REPORT_HINT)
        pass
 
