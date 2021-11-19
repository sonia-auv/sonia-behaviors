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
from sonia_flexbe_states.trick_shot import trick_shot
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 15 2021
@author: FA
'''
class move_to_gateSM(Behavior):
	'''
	Mouvement to gate with trickshot
	'''


	def __init__(self):
		super(move_to_gateSM, self).__init__()
		self.name = 'move_to_gate'

		# parameters of this behavior
		self.add_parameter('distance_to_gate', 2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:846 y:87, x:398 y:199
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:111 y:68
			OperatableStateMachine.add('pose gate',
										create_pose(positionX=self.distance_to_gate, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=25, precision=0, rotation=True),
										transitions={'continue': 'move gate'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'gate_pose'})

			# x:597 y:74
			OperatableStateMachine.add('trickshot',
										trick_shot(delay=3),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:354 y:71
			OperatableStateMachine.add('move gate',
										move_to_target(),
										transitions={'continue': 'trickshot', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'gate_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
