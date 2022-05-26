#!/usr/bin/env python 
#-*- coding: utf-8 -*-

import subprocess, shlex, os

from flexbe_core import EventState, Logger

class start_rosbag_record(EventState):
    '''
        Start the rosbag entered
        [...]

        -- topic_name           string      Topic to record, -a for all (can have more than one, just add a space between)
        -- timer_split          uint8       Max seconds before splitting bags, 0 for no split
        -- bag_name             string      Name for bag(s), 0 for default
        -- record_path          string      Path to record bag(s), 0 to stay here

        #> rosbag_proc          string      Record process to kill after bag is done
        #> command              string      Command used to start the bag

        <= continue			Indicates that the record has started
        <= failed			Indicates that the record didn't start

    '''
    def __init__(self, bag_name, topic_name='/', timer_split=0, record_path='/home/sonia/ssd/Bags'):
        
        super(start_rosbag_record, self).__init__(outcomes = ['continue', 'failed'],
                                                  output_keys = ['rosbag_proc','command'])

        if topic_name == "":
            Logger.log('Topic not set, no rosbag will be created')
            return 'failed'
        else:
            self.topic_name = topic_name

        if bag_name != "0":
            self.bag_name = " -O " + bag_name
        else:
            self.bag_name = ""

        if timer_split != 0:
            self.timer_split = " --split --duration=" + str(timer_split)
        else:
            self.timer_split = ""
        
        if record_path != "0":
            self.record_path = record_path
        else:
            self.record_path = os.getcwd()

        
    def on_enter(self, userdata):
        self.command = "rosbag record" + self.bag_name + " " + self.timer_split + " " + self.topic_name
        Logger.log("%s" %self.command, Logger.REPORT_HINT)
        self.command = shlex.split(self.command)
        Logger.log("%s" %self.command, Logger.REPORT_HINT)

    def execute(self, userdata):

        try:
            os.chdir(self.record_path)
        except:
            Logger.log('Path did not exist, folder will be created', Logger.REPORT_HINT)
            os.mkdir(self.record_path)
            os.chdir(self.record_path)

        try:
            self.rosbag_proc = subprocess.Popen(self.command)
        except:
            return 'failed'
        return 'continue'

    def on_exit(self, userdata):
        userdata.rosbag_proc = self.rosbag_proc
        userdata.command = self.command
        Logger.log('Rosbag started at %s' %self.record_path, Logger.REPORT_HINT)
        pass
 
