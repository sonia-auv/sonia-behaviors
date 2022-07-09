#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_navigation_states.add_pose_to_trajectory import add_pose_to_trajectory
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Feb 02 2022
@author: FA
'''
class moveSM(Behavior):
	'''
	Behavior to move the submarine with a single pose
	'''


	def __init__(self):
		super(moveSM, self).__init__()
		self.name = 'move'

		# parameters of this behavior
		self.add_parameter('positionX', 0)
		self.add_parameter('positionY', 0)
		self.add_parameter('positionZ', 0)
		self.add_parameter('orientationX', 0)
		self.add_parameter('orientationY', 0)
		self.add_parameter('orientationZ', 0)
		self.add_parameter('frame', 1)
		self.add_parameter('speed', 0)
		self.add_parameter('precision', 0)
		self.add_parameter('rotation', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:472 y:473, x:460 y:310
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:69 y:82
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'add_move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:115 y:386
			OperatableStateMachine.add('send',
										send_to_planner(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'move'})

			# x:63 y:227
			OperatableStateMachine.add('add_move',
										add_pose_to_trajectory(positionX=self.positionX, positionY=self.positionY, positionZ=self.positionZ, orientationX=self.orientationX, orientationY=self.orientationY, orientationZ=self.orientationZ, frame=self.frame, speed=self.speed, precision=self.precision, long_rotation=False),
										transitions={'continue': 'send'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'move'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
