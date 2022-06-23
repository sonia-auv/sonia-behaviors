#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.move_to_gate_no_trickshot_sm import move_to_gate_no_trickshotSM
from sonia_flexbe_states.move_to_target import move_to_target
from sonia_navigation_states.set_control_mode import set_control_mode
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu May 31 2022
@author: Guilhem Schena
'''
class gate_no_trickshot_task_v2SM(Behavior):
	'''
	Init the submarine and move through the gate.
	'''


	def __init__(self):
		super(gate_no_trickshot_task_v2SM, self).__init__()
		self.name = 'gate_no_trickshot_task_v2'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(move_to_gate_no_trickshotSM, 'move_to_gate_no_trickshot')

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
			# x:98 y:132
			OperatableStateMachine.add('set control mode',
										set_control_mode(mode=11, timeout=5),
										transitions={'continue': 'move_to_gate_no_trickshot', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:645 y:286
			OperatableStateMachine.add('move_to_gate_no_trickshot',
										self.use_behavior(move_to_gate_no_trickshotSM, 'move_to_gate_no_trickshot'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:657 y:560
			OperatableStateMachine.add('move_buffer',
										move_to_target(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'buffer_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
