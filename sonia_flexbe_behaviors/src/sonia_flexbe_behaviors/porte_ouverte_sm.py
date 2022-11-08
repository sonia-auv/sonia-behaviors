#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.drop_auv8_sm import drop_AUV8SM
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_flexbe_behaviors.torpedoes_sm import torpedoesSM
from sonia_flexbe_behaviors.vision_bins_sm import vision_binsSM
from sonia_flexbe_behaviors.vision_buoys_new_sm import vision_buoys_newSM
from sonia_flexbe_behaviors.vision_path_sm import vision_pathSM
from sonia_flexbe_behaviors.vision_tables_sm import vision_tablesSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Nov 5 2022
@author: CS
'''
class PORTE_OUVERTESM(Behavior):
	'''
	Simulation porte ouverte
	'''


	def __init__(self):
		super(PORTE_OUVERTESM, self).__init__()
		self.name = 'PORTE_OUVERTE'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(drop_AUV8SM, 'drop_AUV8')
		self.add_behavior(moveSM, 'move')
		self.add_behavior(moveSM, 'move_4')
		self.add_behavior(moveSM, 'move_5')
		self.add_behavior(moveSM, 'move_6')
		self.add_behavior(moveSM, 'move_after_buoy')
		self.add_behavior(moveSM, 'move_after_path_2')
		self.add_behavior(moveSM, 'move_to_buoy')
		self.add_behavior(torpedoesSM, 'torpedoes')
		self.add_behavior(vision_binsSM, 'vision_bins')
		self.add_behavior(vision_buoys_newSM, 'vision_buoys_new')
		self.add_behavior(vision_pathSM, 'vision_path')
		self.add_behavior(vision_pathSM, 'vision_path_2')
		self.add_behavior(vision_tablesSM, 'vision_tables')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1250 y:33, x:1163 y:28
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:1255 y:694
			OperatableStateMachine.add('move_6',
										self.use_behavior(moveSM, 'move_6',
											parameters={'positionX': -10, 'positionY': 12, 'positionZ': 1, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 110, 'frame': 0, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'torpedoes', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:923 y:786
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': -12, 'positionY': 0, 'positionZ': 1, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 163, 'frame': 0, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'vision_tables', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:183 y:23
			OperatableStateMachine.add('move_4',
										self.use_behavior(moveSM, 'move_4',
											parameters={'positionX': 0, 'positionY': 0, 'positionZ': 1, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 135, 'frame': 0, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'vision_path', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:412 y:770
			OperatableStateMachine.add('move_5',
										self.use_behavior(moveSM, 'move_5',
											parameters={'positionX': 0, 'positionY': 0.2, 'positionZ': 0.5, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'drop_AUV8', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:182 y:435
			OperatableStateMachine.add('move_after_buoy',
										self.use_behavior(moveSM, 'move_after_buoy',
											parameters={'positionX': -5, 'positionY': 15, 'positionZ': 1, 'orientationX': 0, 'orientationY': 0, 'orientationZ': -155, 'frame': 0, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'vision_path_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:162 y:647
			OperatableStateMachine.add('move_after_path_2',
										self.use_behavior(moveSM, 'move_after_path_2',
											parameters={'positionX': 14, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'vision_bins', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:208 y:227
			OperatableStateMachine.add('move_to_buoy',
										self.use_behavior(moveSM, 'move_to_buoy',
											parameters={'positionX': 5, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'vision_buoys_new', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1294 y:208
			OperatableStateMachine.add('torpedoes',
										self.use_behavior(torpedoesSM, 'torpedoes'),
										transitions={'finished': 'move_4', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:126 y:758
			OperatableStateMachine.add('vision_bins',
										self.use_behavior(vision_binsSM, 'vision_bins',
											parameters={'vision_bins_target': "cover", 'camera_no': 4, 'bounding_box_width': 200, 'bounding_box_height': 200, 'center_bb_height': 50, 'center_bb_width': 50, 'max_mouvement': 1.0, 'min_mouvement': 0.1, 'activate_vision_bins': True}),
										transitions={'finished': 'move_5', 'failed': 'failed', 'lost_target': 'move_4', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:224 y:329
			OperatableStateMachine.add('vision_buoys_new',
										self.use_behavior(vision_buoys_newSM, 'vision_buoys_new',
											parameters={'camera_no': 3, 'vision_buoys_target': "Badge", 'bounding_box_width': 200, 'bounding_box_height': 350, 'center_bounding_box_width': 100, 'center_bounding_box_height': 100, 'max_mouvement': 1.0, 'min_mouvement': 0.25, 'activate_vision_buoys': True, 'vision_buoys_distance_forward': 2.5}),
										transitions={'finished': 'move_after_buoy', 'failed': 'failed', 'lost_target': 'move_4'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:200 y:132
			OperatableStateMachine.add('vision_path',
										self.use_behavior(vision_pathSM, 'vision_path',
											parameters={'vision_path_target': "pipe straight", 'camera_no': 4, 'min_mouvement': 0.25, 'max_mouvement': 1.2, 'bounding_box_height': 100, 'bounding_box_width': 30, 'center_bounding_box_height': 150, 'center_bounding_box_width': 150, 'activate_vision_path': True}),
										transitions={'finished': 'move_to_buoy', 'failed': 'failed', 'lost_target': 'move_4'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'angle': 'angle', 'camera': 'camera'})

			# x:167 y:536
			OperatableStateMachine.add('vision_path_2',
										self.use_behavior(vision_pathSM, 'vision_path_2',
											parameters={'vision_path_target': "pipe straight", 'camera_no': 4, 'min_mouvement': 0.25, 'max_mouvement': 1.2, 'bounding_box_height': 100, 'bounding_box_width': 30, 'center_bounding_box_height': 150, 'center_bounding_box_width': 150}),
										transitions={'finished': 'move_after_path_2', 'failed': 'failed', 'lost_target': 'move_4'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'angle': 'angle', 'camera': 'camera'})

			# x:1235 y:796
			OperatableStateMachine.add('vision_tables',
										self.use_behavior(vision_tablesSM, 'vision_tables',
											parameters={'camera_no': 4, 'bounding_box_width': 10, 'bounding_box_height': 10, 'center_bb_height': 90, 'center_bb_width': 90, 'max_mouvement': 1.0, 'min_mouvement': 0.5, 'activate_vision_tables': True}),
										transitions={'finished': 'move_6', 'failed': 'failed', 'lost_target': 'move_4', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:696 y:757
			OperatableStateMachine.add('drop_AUV8',
										self.use_behavior(drop_AUV8SM, 'drop_AUV8',
											parameters={'activate_drop_auv8': True}),
										transitions={'finished': 'move', 'failed': 'move'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
