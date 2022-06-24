#!/usr/bin/env python 
#-*- coding: utf-8 -*-

# standars includes
from time import time
import rospy
import math
import cmath

# Custom includes
import sonia_navigation_states.modules.navigation_utilities as navUtils

# flexbe includes
from flexbe_core import EventState, Logger
from sonia_common.msg import AddPose, MultiAddPose

class yaw_orbit_from_given_point(EventState):

    '''
    This state generate a 2D orbit in the xy plane arround a given point.

    Parameters:
        pointX and pointY : orbit point expressed in body frame (geometric center of the sub).
        Rotation : amount in degrees to turn arround the point. Positive value turn clockwise and negative value turn counterclockwise
        Speed : Speed profile 

    Essential orbit point.
        bottom AUV8 : [0.285, 0]
        bottom AUV7 : [0.16818, 0]
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

    def __init__(self, pointX=5, pointY=5, rotation = 360, speed =1 ):
        
        super(yaw_orbit_from_given_point, self).__init__(outcomes=['continue'],
                                                     input_keys=['input_traj'],
                                                     output_keys=['trajectory'])

        self.px = pointX
        self.py = pointY
        self.rotation = rotation
        self.speed = speed


    def execute(self, userdata):

        # define msg object
        traj = userdata.input_traj
        new_traj = MultiAddPose()
        new_traj.interpolation_method = 1

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

        nFullPoints = int(math.floor(abs(self.rotation) / degstep))
        residueDegStep = abs(self.rotation) % degstep
        residueRadStep = (residueDegStep*2*math.pi) / 360.0 

        # get direction (cw vs ccw)
        if self.rotation < 0:
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

            new_traj.pose.append(navUtils.addpose(pTrans.real, pTrans.imag, 0, 0, 0, signe * degstep, 1, 0, 0,False))

        # add residue point if needed,
        if residueDegStep > 0 :
            residueTrans= complex( math.cos(residueRadStep), signe * math.sin(residueRadStep))
            residuePoint = (p0 * residueTrans) - p0 # turn p0 of radstep radians and express the rotation in the body frame

            new_traj.pose.append(navUtils.addpose(residuePoint.real, residuePoint.imag, 0, 0, 0, signe * residueDegStep , 1, 0, 0,False))



        # print debug
        Logger.log('Yaw orbit as been succesfully generated ', Logger.REPORT_HINT)

        userdata.trajectory = new_traj
        return 'continue'

    def on_exit(self, userdata):
        pass