#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.create_pose import create_pose
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 11 2021
@author: FA
'''
class move_to_gateSM(Behavior):
	'''
	Movement to the gate in simulation
	'''


	def __init__(self):
		super(move_to_gateSM, self).__init__()
		self.name = 'move_to_gate'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:462 y:180, x:114 y:315
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:54 y:104
			OperatableStateMachine.add('pose gate',
										create_pose(positionX=10, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=20, precision=0, rotation=True),
										transitions={'continue': 'move gate'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'gate_pose'})

			# x:263 y:210
			OperatableStateMachine.add('move gate',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'gate_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
