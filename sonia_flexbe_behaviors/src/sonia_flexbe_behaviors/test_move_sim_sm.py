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
from sonia_flexbe_states.set_control_mode import set_control_mode
from sonia_flexbe_states.set_initial_position import set_initial_position
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 10 2021
@author: FA
'''
class test_move_simSM(Behavior):
	'''
	Test movement for simulation
	'''


	def __init__(self):
		super(test_move_simSM, self).__init__()
		self.name = 'test_move_sim'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:820 y:388, x:268 y:463
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:99 y:95
			OperatableStateMachine.add('asdf',
										set_control_mode(mode=32, timeout=3),
										transitions={'continue': 'start', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:587 y:134
			OperatableStateMachine.add('create_init',
										create_pose(positionX=2, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, time=5, precision=0, rotation=True),
										transitions={'continue': 'move_to_target'},
										autonomy={'continue': Autonomy.Off},
										remapping={'pose': 'initial_pose'})

			# x:588 y:286
			OperatableStateMachine.add('move_to_target',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'initial_pose'})

			# x:364 y:112
			OperatableStateMachine.add('start',
										set_initial_position(),
										transitions={'continue': 'create_init'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
