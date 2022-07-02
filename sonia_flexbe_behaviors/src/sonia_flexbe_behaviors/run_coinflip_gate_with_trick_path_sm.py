#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.coin_flip_gate__notrickshot_task_sm import coin_flip_gate_notrickshot_taskSM
from sonia_flexbe_behaviors.init_submarine_sm import init_submarineSM
from sonia_flexbe_behaviors.trickshot_sm import trickshotSM
from sonia_flexbe_behaviors.vision_path_new_algo_sm import vision_path_new_algoSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 23 2022
@author: GS
'''
class Run_coinflip_gate_with_trick_pathSM(Behavior):
	'''
	Run coinflip, gate without trickshot and path
	'''


	def __init__(self):
		super(Run_coinflip_gate_with_trick_pathSM, self).__init__()
		self.name = 'Run_coinflip_gate_with_trick_path'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(coin_flip_gate_notrickshot_taskSM, 'coin_flip_gate_ notrickshot_task')
		self.add_behavior(init_submarineSM, 'init_submarine')
		self.add_behavior(trickshotSM, 'trickshot')
		self.add_behavior(vision_path_new_algoSM, 'vision_path_new_algo')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1151 y:83, x:465 y:364
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:31 y:72
			OperatableStateMachine.add('init_submarine',
										self.use_behavior(init_submarineSM, 'init_submarine'),
										transitions={'finished': 'coin_flip_gate_ notrickshot_task', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:656 y:69
			OperatableStateMachine.add('trickshot',
										self.use_behavior(trickshotSM, 'trickshot'),
										transitions={'finished': 'vision_path_new_algo', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:922 y:68
			OperatableStateMachine.add('vision_path_new_algo',
										self.use_behavior(vision_path_new_algoSM, 'vision_path_new_algo'),
										transitions={'finished': 'finished', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:365 y:64
			OperatableStateMachine.add('coin_flip_gate_ notrickshot_task',
										self.use_behavior(coin_flip_gate_notrickshot_taskSM, 'coin_flip_gate_ notrickshot_task'),
										transitions={'finished': 'trickshot', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
