#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.droppers_task_sm import droppers_taskSM
from sonia_flexbe_behaviors.gate_task_sm import gate_taskSM
from sonia_flexbe_behaviors.jiangshi_task_sm import jiangshi_taskSM
from sonia_flexbe_behaviors.path_task_sm import path_taskSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 17 2021
@author: William Brouillard
'''
class Competition_runSM(Behavior):
	'''
	Task executed in the 2021 QC competition. 
Gate, path, Jiangshi, path, droppers.
	'''


	def __init__(self):
		super(Competition_runSM, self).__init__()
		self.name = 'Competition_run'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(droppers_taskSM, 'droppers_task')
		self.add_behavior(gate_taskSM, 'gate_task')
		self.add_behavior(jiangshi_taskSM, 'jiangshi_task')
		self.add_behavior(path_taskSM, 'path_task_1')
		self.add_behavior(path_taskSM, 'path_task_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:546 y:682, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:89 y:69
			OperatableStateMachine.add('gate_task',
										self.use_behavior(gate_taskSM, 'gate_task'),
										transitions={'finished': 'path_task_1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:500 y:276
			OperatableStateMachine.add('jiangshi_task',
										self.use_behavior(jiangshi_taskSM, 'jiangshi_task'),
										transitions={'finished': 'path_task_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:360 y:130
			OperatableStateMachine.add('path_task_1',
										self.use_behavior(path_taskSM, 'path_task_1'),
										transitions={'finished': 'jiangshi_task', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:503 y:400
			OperatableStateMachine.add('path_task_2',
										self.use_behavior(path_taskSM, 'path_task_2'),
										transitions={'finished': 'droppers_task', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:498 y:541
			OperatableStateMachine.add('droppers_task',
										self.use_behavior(droppers_taskSM, 'droppers_task'),
										transitions={'finished': 'finished', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
