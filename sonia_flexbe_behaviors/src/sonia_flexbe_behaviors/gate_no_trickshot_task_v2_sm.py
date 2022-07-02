#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.move_to_gate_no_trickshot_v2_sm import move_to_gate_no_trickshot_v2SM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu May 31 2022
@author: Guilhem Schena
'''
class gate_no_trickshot_task_v2SM(Behavior):
	'''
	Move through the gate, no trickshot
	'''


	def __init__(self):
		super(gate_no_trickshot_task_v2SM, self).__init__()
		self.name = 'gate_no_trickshot_task_v2'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(move_to_gate_no_trickshot_v2SM, 'move_to_gate_no_trickshot_v2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:950 y:89, x:465 y:590
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:460 y:89
			OperatableStateMachine.add('move_to_gate_no_trickshot_v2',
										self.use_behavior(move_to_gate_no_trickshot_v2SM, 'move_to_gate_no_trickshot_v2'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
