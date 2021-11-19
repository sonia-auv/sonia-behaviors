#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.coin_flip_sm import coin_flipSM
from sonia_flexbe_behaviors.init_submarine_sm import init_submarineSM
from sonia_flexbe_behaviors.move_to_gate_sm import move_to_gateSM
from sonia_flexbe_states.create_pose import create_pose
from sonia_flexbe_states.move_to_target import move_to_target
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Nov 12 2021
@author: William Brouillard
'''
class gate_taskSM(Behavior):
	'''
	Init the submarine, orient to the gate with the right orientation and move through it with a trickshot.
	'''


	def __init__(self):
		super(gate_taskSM, self).__init__()
		self.name = 'gate_task'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(coin_flipSM, 'coin_flip')
		self.add_behavior(init_submarineSM, 'init_submarine')
		self.add_behavior(move_to_gateSM, 'move_to_gate')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:878 y:587, x:465 y:590
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:137 y:90
			OperatableStateMachine.add('buffer_pose',
										create_pose(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=5, precision=0, rotation=True),
										transitions={'continue': 'init_submarine'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'buffer_pose'})

			# x:417 y:280
			OperatableStateMachine.add('coin_flip',
										self.use_behavior(coin_flipSM, 'coin_flip'),
										transitions={'finished': 'move_to_gate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:128 y:271
			OperatableStateMachine.add('init_submarine',
										self.use_behavior(init_submarineSM, 'init_submarine'),
										transitions={'finished': 'coin_flip', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:657 y:560
			OperatableStateMachine.add('move_buffer',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'buffer_pose'})

			# x:684 y:278
			OperatableStateMachine.add('move_to_gate',
										self.use_behavior(move_to_gateSM, 'move_to_gate'),
										transitions={'finished': 'move_buffer', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
