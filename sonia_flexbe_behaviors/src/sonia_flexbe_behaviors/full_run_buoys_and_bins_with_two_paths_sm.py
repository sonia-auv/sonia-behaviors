#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.avoid_buoys_after_collision_sm import avoid_buoys_after_collisionSM
from sonia_flexbe_behaviors.drop_auv7_sm import drop_AUV7SM
from sonia_flexbe_behaviors.vision_bins_sm import vision_binsSM
from sonia_flexbe_behaviors.vision_buoys_sm import vision_buoysSM
from sonia_flexbe_behaviors.vision_droppers_sm import vision_droppersSM
from sonia_flexbe_behaviors.vision_path_sm import vision_pathSM
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
		self.add_behavior(avoid_buoys_after_collisionSM, 'avoid_buoys_after_collision')
		self.add_behavior(drop_AUV7SM, 'drop_AUV7')
		self.add_behavior(vision_binsSM, 'vision_bins')
		self.add_behavior(vision_buoysSM, 'vision_buoys')
		self.add_behavior(vision_droppersSM, 'vision_droppers')
		self.add_behavior(vision_pathSM, 'vision_path')
		self.add_behavior(vision_pathSM, 'vision_path_2')

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
			# x:51 y:60
			OperatableStateMachine.add('vision_path',
										self.use_behavior(vision_pathSM, 'vision_path'),
										transitions={'finished': 'vision_buoys', 'failed': 'failed', 'lost_target': 'failed'},
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

			# x:368 y:61
			OperatableStateMachine.add('vision_buoys',
										self.use_behavior(vision_buoysSM, 'vision_buoys'),
										transitions={'finished': 'avoid_buoys_after_collision', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:944 y:534
			OperatableStateMachine.add('vision_droppers',
										self.use_behavior(vision_droppersSM, 'vision_droppers'),
										transitions={'finished': 'drop_AUV7', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:944 y:213
			OperatableStateMachine.add('vision_path_2',
										self.use_behavior(vision_pathSM, 'vision_path_2'),
										transitions={'finished': 'vision_bins', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:927 y:57
			OperatableStateMachine.add('avoid_buoys_after_collision',
										self.use_behavior(avoid_buoys_after_collisionSM, 'avoid_buoys_after_collision'),
										transitions={'finished': 'vision_path_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
