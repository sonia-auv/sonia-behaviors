#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.drop_auv7_sm import drop_AUV7SM
from sonia_flexbe_behaviors.vision_bins_sm import vision_binsSM
from sonia_flexbe_behaviors.vision_buoy_sm import vision_buoySM
from sonia_flexbe_behaviors.vision_droppers_sm import vision_droppersSM
from sonia_flexbe_behaviors.vision_path_new_algo_sm import vision_path_new_algoSM
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 19 2022
@author: Willy Kao
'''
class full_run_buoys_and_bins_with_two_pathsSM(Behavior):
	'''
	Full-fledged mission run starting from the first path towards the buoys task and then, the AUV leaves for the bins task through the second path.
	'''


	def __init__(self):
		super(full_run_buoys_and_bins_with_two_pathsSM, self).__init__()
		self.name = 'full_run_buoys_and_bins_with_two_paths'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(drop_AUV7SM, 'drop_AUV7')
		self.add_behavior(vision_binsSM, 'vision_bins')
		self.add_behavior(vision_buoySM, 'vision_buoy')
		self.add_behavior(vision_droppersSM, 'vision_droppers')
		self.add_behavior(vision_path_new_algoSM, 'vision_path_new_algo')
		self.add_behavior(vision_path_new_algoSM, 'vision_path_new_algo_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1017 y:823, x:412 y:464
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:92 y:60
			OperatableStateMachine.add('vision_path_new_algo',
										self.use_behavior(vision_path_new_algoSM, 'vision_path_new_algo'),
										transitions={'finished': 'vision_buoy', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:945 y:676
			OperatableStateMachine.add('drop_AUV7',
										self.use_behavior(drop_AUV7SM, 'drop_AUV7'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:944 y:384
			OperatableStateMachine.add('vision_bins',
										self.use_behavior(vision_binsSM, 'vision_bins'),
										transitions={'finished': 'vision_droppers', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:354 y:62
			OperatableStateMachine.add('vision_buoy',
										self.use_behavior(vision_buoySM, 'vision_buoy'),
										transitions={'finished': 'EMPTY', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:944 y:534
			OperatableStateMachine.add('vision_droppers',
										self.use_behavior(vision_droppersSM, 'vision_droppers'),
										transitions={'finished': 'drop_AUV7', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:937 y:227
			OperatableStateMachine.add('vision_path_new_algo_2',
										self.use_behavior(vision_path_new_algoSM, 'vision_path_new_algo_2'),
										transitions={'finished': 'vision_bins', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:957 y:55
			OperatableStateMachine.add('EMPTY',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'vision_path_new_algo_2', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
