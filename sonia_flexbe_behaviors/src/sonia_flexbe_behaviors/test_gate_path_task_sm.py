#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.gate_quick_task_sm import gate_quick_taskSM
from sonia_flexbe_behaviors.path_task_sm import path_taskSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Nov 20 2021
@author: William Brouillard
'''
class Test_gate_path_taskSM(Behavior):
	'''
	Test the gate quick task with the path_task.
	'''


	def __init__(self):
		super(Test_gate_path_taskSM, self).__init__()
		self.name = 'Test_gate_path_task'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(gate_quick_taskSM, 'gate_quick_task')
		self.add_behavior(path_taskSM, 'path_task')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:706 y:77, x:130 y:365, x:516 y:306
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'lost_target'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:90 y:69
			OperatableStateMachine.add('gate_quick_task',
										self.use_behavior(gate_quick_taskSM, 'gate_quick_task'),
										transitions={'finished': 'path_task', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:388 y:69
			OperatableStateMachine.add('path_task',
										self.use_behavior(path_taskSM, 'path_task'),
										transitions={'finished': 'finished', 'failed': 'failed', 'lost_target': 'lost_target'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
