#!/usr/bin/env python 
#-*- coding: utf-8 -*-

# standars includes
from time import time
from xml.dom import UserDataHandler
import rospy
import math
import cmath

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils

# flexbe includes
from flexbe_core import EventState, Logger
from sonia_common.msg import AddPose, MultiAddPose

class yaw_orbit_from_given_point_and_angle(EventState):

    '''
    This state generate a 2D orbit in the xy plane arround a given point.
    Essential orbit point.
        bottom AUV8 : [0.2415, 0]
        bottom AUV7 : [X, X]

        ># input_traj       MultiAddPose    Input trajectory
        ># camera           uint8           0: None
                                            1: bottom AUV8
                                            2: bottom AUV7
        ># angle            float           angle from vision

        #> trajectory       MultiAddPose    Output trajectory

        <= continue                         Send trajectory to planner
    '''
    '''
                                  |<--- pointY --->|
                         _ _                       * <= orbit point
                          ^  
                          |  
                          | 
                        pointX
                          |      ___        ^ x
                          |     | ^ |       |
                          |    _|   |_      |
                         _v_  |_ sub _|     -----> y
                                |   |       body frame
                                |___|

     
        '''

    def __init__(self, pointX=0, pointY=0):
        
        super(yaw_orbit_from_given_point_and_angle, self).__init__(outcomes=['continue'],
                                                     input_keys=['input_traj','camera','angle'],
                                                     output_keys=['trajectory'])
        self.px = pointX
        self.px = pointY

    def on_enter(self, userdata):
        if userdata.camera == 1:
            self.px = 0.2415
            self.py = 0
        elif userdata.camera == 2:
            self.px = 0 # TODO: change
            self.py = 0 # TODO: change


    def execute(self, userdata):

        # define msg object
        traj = userdata.input_traj
        new_traj = MultiAddPose()

        # Add previous waypoint if needed
        if not traj.pose:
            Logger.log('First position of the trajectory', Logger.REPORT_HINT)
        else:
            Logger.log('Adding a pose to the trajectory', Logger.REPORT_HINT)            
            new_traj.pose = list(traj.pose)

        # compute parameter
        ppt = 8 # point per turn
        radstep = (2*math.pi)/ppt 
        degstep = 360.0 / ppt
        Logger.log(degstep, Logger.REPORT_HINT)

        nFullPoints = int(abs(math.ceil(userdata.angle / degstep)))
        Logger.log(nFullPoints, Logger.REPORT_HINT)
        residueDegStep = abs(userdata.angle % degstep)
        residueRadStep = (residueDegStep*2*math.pi) / 360.0 

        # get direction (cw vs ccw)
        if userdata.angle < 0:
            signe = -1
        else:
            signe =  1

        # Express orbit in the complex plane. (easier for calculation) (work on the same principle as quaternions)
        # check complex multiplication for rotation transformation for further details

        p0 = complex(-self.px, -self.py) # initial position of the sub according to the orbit point

        # Full step transformation
        trans = complex( math.cos(radstep), signe * math.sin(radstep)) # rotation of radstep radians
        pTrans = (p0 * trans) - p0 # turn p0 of radstep radians and express the rotation in the body frame

        # Generate the require amount of addpose msg
        for i in range(nFullPoints):
            Logger.log(signe*degstep,Logger.REPORT_HINT)
            new_traj.pose.append(navUtils.addpose(pTrans.real, pTrans.imag, 0, 0, 0, signe * degstep, 1, 0, 0,False))

        # add residue point if needed,
        if residueDegStep > 0 :
            residueTrans= complex( math.cos(residueRadStep), signe * math.sin(residueRadStep))
            residuePoint = (p0 * residueTrans) - p0 # turn p0 of radstep radians and express the rotation in the body frame
            Logger.log(signe * residueDegStep,Logger.REPORT_HINT)
            new_traj.pose.append(navUtils.addpose(residuePoint.real, residuePoint.imag, 0, 0, 0, signe * residueDegStep , 1, 0, 0,False))



        # print debug
        Logger.log('Yaw orbit as been succesfully generated ', Logger.REPORT_HINT)

        userdata.trajectory = new_traj
        return 'continue'

    def on_exit(self, userdata):
        pass