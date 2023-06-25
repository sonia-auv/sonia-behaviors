#!/usr/bin/env python 
#-*- coding: utf-8 -*-

# standars includes
from time import time, sleep
from typing import Optional
import rospy
from dataclasses import dataclass
import math

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils

from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose
from sonia_common.msg import MultiAddPose, AddPose
from tf.transformations import euler_from_quaternion


@dataclass
class Position:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


class save_pose(EventState):

    def __init__(self):
        super(save_pose, self).__init__(
            outcomes=['success', 'fail'],
            input_keys=['input_traj'],
            output_keys=['trajectory'])

        self.position_target_subscriber: Optional[rospy.Subscriber] = None
        self.cur_pose: Position = None

    def on_enter(self, userdata):
        self.position_target_subscriber = rospy.Subscriber('/proc_control/current_target', Pose, self._position_target_callback)
    

    def execute(self, userdata):
        timeout_counter = 0
        
        while self.cur_pose is None:
            sleep(0.1)
            timeout_counter += 1
            if timeout_counter > 50:
                Logger.log('5 seconds, fail', Logger.REPORT_HINT)
                return 'fail'
        traj = userdata.input_traj
        new_traj = MultiAddPose()
        if not traj.pose:
            Logger.log('First position of the trajectory', Logger.REPORT_HINT)
        else:
            Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)            
            new_traj.pose = list(traj.pose)

        new_traj.pose.append(navUtils.addpose(
            x=self.cur_pose.x,
            y=self.cur_pose.y,
            z=self.cur_pose.z,
            rx=self.cur_pose.roll,
            ry=self.cur_pose.pitch,
            rz=self.cur_pose.yaw,
            frame=0,
            speed=0,
            fine=0,
            rot=False
        ))
        userdata.trajectory = new_traj

        return 'success'

    def on_exit(self, userdata):
        self.position_target_subscriber.unregister()
        self.position_target_subscriber = None
        self.cur_pose = None

    def _position_target_callback(self, data):
        try:
            self.cur_pose = Position(
                x=data.position.x,
                y=data.position.y,
                z=data.position.z,
                roll=math.degrees(euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w],'szyx')[2]),
                pitch=math.degrees(euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w],'szyx')[1]),
                yaw=math.degrees(euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w],'szyx')[0]),
            )
        except ValueError:
            self.cur_pose = None
        pass
