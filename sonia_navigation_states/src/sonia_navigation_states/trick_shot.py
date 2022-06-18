#!/usr/bin/env python 
#-*- coding: utf-8 -*-

from time import sleep
import rospy

from flexbe_core import EventState, Logger
from geometry_msgs.msg import Vector3, Point
from std_msgs.msg import Bool
from sonia_common.msg import AddPose

class trick_shot(EventState):
    '''
        Send multiple pose to do a trick shot in gate.
        [...]

        --delay      uint8   Delay wanted at the .

        <= continue     Indicate that trick shot is complete
    '''
    
    def __init__(self, delay):
        
        super(trick_shot, self).__init__(outcomes=['continue'])
        self.add_pose_pub = rospy.Publisher('/proc_control/add_pose', AddPose, queue_size=10)
        self.delay = delay
        self.waypoints = [
                        # Go down:
                        #{ 'position': Point(0, 0, 1), 'orientation': Vector3(0, 0, 0), 'frame': 1, 'speed': 5, 'fine': 0, 'rotation': True }
                        # Funny stuff:
                            # [Point(0, 0, 0),Vector3(0, 55, 0),1,4,0,True],
                            # [Point(0, 0, 0),Vector3(0, 0, 180),1,6,0,True],
			            # { 'position': Point(0, 0, 1), 'orientation': Vector3(0, 0, 0), 'frame': 1, 'speed': 6, 'fine': 0, 'rotation': True },
			            #{ 'position': Point(0, 0, 0), 'orientation': Vector3(0, 0, 180), 'frame': 1, 'speed': 6, 'fine': 0, 'rotation': True },
                            # [Point(0, 0, 0),Vector3(0, 55, 0),1,6,0,True],
                            # [Point(0, 0, 0),Vector3(0, 0, 180),1,5,0,True]]
                            # [Point(0, 0, 0),Vector3(180, -0, 0),1,3,0,True],
                            # [Point(0, 0, 0),Vector3(180, -0, 0),1,3,0,True]]

			#funnystuff2			  
			 #{ 'position': Point(0, 0, 1), 'orientation': Vector3(0, 0, 0), 'frame': 1, 'speed': 6, 'fine': 0, 'rotation': True },
			 #{ 'position': Point(5, 0, 0), 'orientation': Vector3(360, 0, 0), 'frame': 1, 'speed': 6, 'fine': 0, 'rotation': True }

                        # Square:
                          #{ 'position': Point(0, 0, 1), 'orientation': Vector3(0, 0, 0), 'frame': 1, 'speed': 5, 'fine': 0, 'rotation': True },
                          #{ 'position': Point(5, 0, 0), 'orientation': Vector3(0, 0, 0), 'frame': 1, 'speed': 12, 'fine': 0, 'rotation': True },
                          #{ 'position': Point(0, 0, 0), 'orientation': Vector3(0, 0, 90), 'frame': 1, 'speed': 4, 'fine': 0, 'rotation': True },
                          #{ 'position': Point(0, 5, 0), 'orientation': Vector3(0, 0, 0), 'frame': 1, 'speed': 12, 'fine': 0, 'rotation': True },
                          #{ 'position': Point(0, 0, 0), 'orientation': Vector3(0, 0, 90), 'frame': 1, 'speed': 4, 'fine': 0, 'rotation': True },
                          #{ 'position': Point(-5, 0, 0), 'orientation': Vector3(0, 0, 0), 'frame': 1, 'speed': 12, 'fine': 0, 'rotation': True },
                          #{ 'position': Point(0, 0, 0), 'orientation': Vector3(0, 0, 90), 'frame': 1, 'speed': 4, 'fine': 0, 'rotation': True },
                          #{ 'position': Point(0, -5, 0), 'orientation': Vector3(0, 0, 0), 'frame': 1, 'speed': 12, 'fine': 0, 'rotation': True },
                          #{ 'position': Point(0, 0, 0), 'orientation': Vector3(0, 0, 180), 'frame': 1, 'speed': 4, 'fine': 0, 'rotation': True }

                        # Barrel Roll
                          { 'position': Point(0, 0, 1.5), 'orientation': Vector3(0, 0, 0), 'frame': 1, 'speed': 5, 'fine': 0, 'rotation': True },
                          { 'position': Point(0, 0, 0), 'orientation': Vector3(180, 0, 0), 'frame': 1, 'speed': 6, 'fine': 0, 'rotation': True },
                          { 'position': Point(0, 0, 0), 'orientation': Vector3(180, 0, 0), 'frame': 1, 'speed': 6, 'fine': 0, 'rotation': True }]                     
        sleep(1)

    def target_reached_cb(self, data):
        self.target_reach = data.data

    def start(self):
        Logger.log('Trajecotry started', Logger.REPORT_HINT)
        for wpt in self.waypoints:
            self.add_pose_pub.publish(position=wpt[0], orientation=wpt[1], frame=wpt[2], speed=wpt[3], fine=wpt[4])
            sleep(wpt[3])
        sleep(self.delay)
        Logger.log('Trajectory finished', Logger.REPORT_HINT)

    def on_enter(self, userdata):
        self.target_reach = False
        self.start()
        self.target_reached = rospy.Subscriber('/proc_control/target_reached', Bool, self.target_reached_cb)

    def execute(self, userdata):
        if self.target_reach == True:
            return 'continue'

    def on_exit(self, userdata):
        pass
