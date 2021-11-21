#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.gate_no_trickshot_task_sm import gate_no_trickshot_taskSM
from sonia_flexbe_behaviors.jiangshi_task_sm import jiangshi_taskSM
from sonia_flexbe_behaviors.path_task_sm import path_taskSM
from sonia_flexbe_behaviors.vision_droppers_sm import vision_droppersSM
from sonia_flexbe_states.set_control_mode import set_control_mode
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Nov 17 2021
@author: William FA
'''
class CompetitionrunCSM(Behavior):
	'''
	Gate no trickshot task
Path task
Jiangshi task
Path task
Dropper task
	'''


	def __init__(self):
		super(CompetitionrunCSM, self).__init__()
		self.name = 'Competition run C'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(gate_no_trickshot_taskSM, 'gate_no_trickshot_task')
		self.add_behavior(jiangshi_taskSM, 'jiangshi_task')
		self.add_behavior(path_taskSM, 'path_task')
		self.add_behavior(path_taskSM, 'path_task_2')
		self.add_behavior(vision_droppersSM, 'vision_droppers')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:893 y:586, x:41 y:477
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:82 y:57
			OperatableStateMachine.add('gate_no_trickshot_task',
										self.use_behavior(gate_no_trickshot_taskSM, 'gate_no_trickshot_task'),
										transitions={'finished': 'path_task', 'failed': 'stop control 2'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:445 y:198
			OperatableStateMachine.add('jiangshi_task',
										self.use_behavior(jiangshi_taskSM, 'jiangshi_task'),
										transitions={'finished': 'path_task_2', 'failed': 'stop control 2'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:333 y:75
			OperatableStateMachine.add('path_task',
										self.use_behavior(path_taskSM, 'path_task'),
										transitions={'finished': 'jiangshi_task', 'failed': 'stop control 2', 'lost_target': 'stop control 2'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:462 y:328
			OperatableStateMachine.add('path_task_2',
										self.use_behavior(path_taskSM, 'path_task_2'),
										transitions={'finished': 'vision_droppers', 'failed': 'stop control 2', 'lost_target': 'stop control 2'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:627 y:571
			OperatableStateMachine.add('stop control 1',
										set_control_mode(mode=0, timeout=3),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:159 y:325
			OperatableStateMachine.add('stop control 2',
										set_control_mode(mode=0, timeout=3),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:472 y:455
			OperatableStateMachine.add('vision_droppers',
										self.use_behavior(vision_droppersSM, 'vision_droppers'),
										transitions={'finished': 'stop control 1', 'failed': 'stop control 2', 'lost_target': 'stop control 2'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
