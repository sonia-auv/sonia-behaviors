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
from sonia_flexbe_behaviors.vision_torpedoes_boards_sm import vision_torpedoes_boardsSM
from sonia_flexbe_behaviors.vision_torpedoes_sm import vision_torpedoesSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 27 2022
@author: GS
'''
class torpedoesSM(Behavior):
	'''
	align and shoot
	'''


	def __init__(self):
		super(torpedoesSM, self).__init__()
		self.name = 'torpedoes'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(launch_AUV8SM, 'launch_AUV8')
		self.add_behavior(moveSM, 'move')
		self.add_behavior(vision_torpedoesSM, 'vision_torpedoes')
		self.add_behavior(vision_torpedoes_boardsSM, 'vision_torpedoes_boards')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:882 y:317, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('vision_torpedoes_boards',
										self.use_behavior(vision_torpedoes_boardsSM, 'vision_torpedoes_boards',
											parameters={'vision_torpedoes_boards_target': "G-Man", 'bounding_box_width': 150, 'bounding_box_height': 200, 'center_bounding_box_width': 100, 'center_bounding_box_height': 100, 'max_mouvement': 2, 'min_mouvement': 0.25, 'activate_vision_buoys': True}),
										transitions={'finished': 'move', 'failed': 'failed', 'lost_target': 'move'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:287 y:50
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': 1, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'vision_torpedoes', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:507 y:109
			OperatableStateMachine.add('vision_torpedoes',
										self.use_behavior(vision_torpedoesSM, 'vision_torpedoes',
											parameters={'torpedoes_target': "torpedoes", 'torpedoes_bounding_box_width': 300, 'torpedoes_bounding_box_height': 300, 'torpedoes_center_bounding_box_height': 100, 'torpedoes_center_bounding_box_width': 100, 'torpedoes_max_mouv': 0.5, 'torpedoes_min_mouv': 0.1}),
										transitions={'finished': 'launch_AUV8', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:677 y:263
			OperatableStateMachine.add('launch_AUV8',
										self.use_behavior(launch_AUV8SM, 'launch_AUV8',
											parameters={'activate_launch_auv8': True, 'launch_x': 0.3, 'launch_y': 0.07, 'launch_z': -0.1}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
