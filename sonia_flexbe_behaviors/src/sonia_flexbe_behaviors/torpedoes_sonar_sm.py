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
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_flexbe_behaviors.sonar_end_2_end_sm import sonar_end_2_endSM
from sonia_flexbe_behaviors.vision_torpedoes_sm import vision_torpedoesSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 27 2022
@author: GS
'''
class torpedoes_sonarSM(Behavior):
	'''
	align with sonar and shoot
	'''


	def __init__(self):
		super(torpedoes_sonarSM, self).__init__()
		self.name = 'torpedoes_sonar'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(launch_AUV8SM, 'launch_AUV8')
		self.add_behavior(moveSM, 'move')
		self.add_behavior(sonar_end_2_endSM, 'sonar_end_2_end')
		self.add_behavior(vision_torpedoesSM, 'vision_torpedoes')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1146 y:266, x:381 y:261
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:77 y:47
			OperatableStateMachine.add('sonar_end_2_end',
										self.use_behavior(sonar_end_2_endSM, 'sonar_end_2_end'),
										transitions={'finished': 'move', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:354 y:49
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': 1, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'vision_torpedoes', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:645 y:50
			OperatableStateMachine.add('vision_torpedoes',
										self.use_behavior(vision_torpedoesSM, 'vision_torpedoes',
											parameters={'torpedoes_target': "torpedoes", 'torpedoes_bounding_box_width': 300, 'torpedoes_bounding_box_height': 300, 'torpedoes_center_bounding_box_height': 100, 'torpedoes_center_bounding_box_width': 100, 'torpedoes_max_mouv': 0.5, 'torpedoes_min_mouv': 0.1}),
										transitions={'finished': 'launch_AUV8', 'failed': 'failed', 'lost_target': 'move', 'controller_error': 'move'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:844 y:254
			OperatableStateMachine.add('launch_AUV8',
										self.use_behavior(launch_AUV8SM, 'launch_AUV8',
											parameters={'activate_launch_auv8': True, 'launch_x': 0.3, 'launch_y': 0.07, 'launch_z': -0.1}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
