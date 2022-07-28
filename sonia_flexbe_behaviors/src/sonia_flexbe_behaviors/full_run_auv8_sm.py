#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.launch_auv8_sm import launch_AUV8SM
from sonia_flexbe_behaviors.vision_buoys_sm import vision_buoysSM
from sonia_flexbe_behaviors.vision_droppers_sm import vision_droppersSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on 27/08/2022
@author: GS
'''
class full_run_AUV8SM(Behavior):
	'''
	Missions for AUV8 : align torpedoes and launch torpedoes
	'''


	def __init__(self):
		super(full_run_AUV8SM, self).__init__()
		self.name = 'full_run_AUV8'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(launch_AUV8SM, 'launch_AUV8')
		self.add_behavior(vision_buoysSM, 'vision_buoys')
		self.add_behavior(vision_droppersSM, 'vision_droppers')

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
			# x:934 y:299
			OperatableStateMachine.add('vision_buoys',
										self.use_behavior(vision_buoysSM, 'vision_buoys'),
										transitions={'finished': 'vision_droppers', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:944 y:534
			OperatableStateMachine.add('vision_droppers',
										self.use_behavior(vision_droppersSM, 'vision_droppers',
											parameters={'camera_no': 2, 'center_bounding_box_height': 50, 'center_bounding_box_width': 50}),
										transitions={'finished': 'launch_AUV8', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:959 y:682
			OperatableStateMachine.add('launch_AUV8',
										self.use_behavior(launch_AUV8SM, 'launch_AUV8'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
