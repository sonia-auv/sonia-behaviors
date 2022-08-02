#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.coinflip_gate_trickshot_with_com_sm import CoinFlipGateTrickshotwithcomSM
from sonia_flexbe_behaviors.drop_auv7_sm import drop_AUV7SM
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_flexbe_behaviors.move_with_detection_buoys_sm import move_with_detection_buoysSM
from sonia_flexbe_behaviors.square_sm import squareSM
from sonia_flexbe_behaviors.vision_bins_sm import vision_binsSM
from sonia_flexbe_behaviors.vision_buoys_collision_detector_sm import vision_buoys_collision_detectorSM
from sonia_flexbe_behaviors.vision_droppers_sm import vision_droppersSM
from sonia_flexbe_behaviors.vision_path_sm import vision_pathSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 30 2022
@author: CS
'''
class AUV7_FINALSM(Behavior):
	'''
	AUV7_FINAL
	'''


	def __init__(self):
		super(AUV7_FINALSM, self).__init__()
		self.name = 'AUV7_FINAL'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com')
		self.add_behavior(drop_AUV7SM, 'drop_AUV7')
		self.add_behavior(moveSM, 'move')
		self.add_behavior(moveSM, 'move_2')
		self.add_behavior(move_with_detection_buoysSM, 'move_with_detection_buoys')
		self.add_behavior(squareSM, 'square')
		self.add_behavior(vision_binsSM, 'vision_bins')
		self.add_behavior(vision_buoys_collision_detectorSM, 'vision_buoys_collision_detector')
		self.add_behavior(vision_droppersSM, 'vision_droppers')
		self.add_behavior(vision_droppersSM, 'vision_droppers_2')
		self.add_behavior(vision_pathSM, 'vision_path')
		self.add_behavior(vision_pathSM, 'vision_path_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1279 y:381, x:1163 y:28
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:171 y:31
			OperatableStateMachine.add('CoinFlip-Gate-Trickshot with com',
										self.use_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com',
											parameters={'submarine': "AUV7", 'dive_depth': 1.5, 'has_com': False, 'activate_coinflip_gate_trickshot_com': True}),
										transitions={'finished': 'vision_path', 'failed': 'failed', 'failed_start_control': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'failed_start_control': Autonomy.Inherit})

			# x:624 y:757
			OperatableStateMachine.add('drop_AUV7',
										self.use_behavior(drop_AUV7SM, 'drop_AUV7',
											parameters={'activate_drop_auv7': True}),
										transitions={'finished': 'move', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:937 y:630
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': 0, 'positionY': 0, 'positionZ': 0.3, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'square', 'failed': 'square'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:4 y:55
			OperatableStateMachine.add('move_2',
										self.use_behavior(moveSM, 'move_2',
											parameters={'orientationZ': 180}),
										transitions={'finished': 'vision_path_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:194 y:251
			OperatableStateMachine.add('move_with_detection_buoys',
										self.use_behavior(move_with_detection_buoysSM, 'move_with_detection_buoys',
											parameters={'move_for_buoys': 5}),
										transitions={'finished': 'vision_buoys_collision_detector', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1010 y:451
			OperatableStateMachine.add('square',
										self.use_behavior(squareSM, 'square',
											parameters={'box_size': 3, 'stroke': 0.3}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:205 y:590
			OperatableStateMachine.add('vision_bins',
										self.use_behavior(vision_binsSM, 'vision_bins',
											parameters={'vision_bins_target': "cover", 'bounding_box_width': 200, 'bounding_box_height': 200, 'center_bb_height': 50, 'center_bb_width': 50, 'max_mouvement': 1.0, 'min_mouvement': 0.1, 'activate_vision_bins': True}),
										transitions={'finished': 'vision_droppers', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:177 y:345
			OperatableStateMachine.add('vision_buoys_collision_detector',
										self.use_behavior(vision_buoys_collision_detectorSM, 'vision_buoys_collision_detector',
											parameters={'vision_buoys_target': "Badge", 'bounding_box_width': 150, 'bounding_box_height': 150, 'center_bounding_box_width': 100, 'center_bounding_box_height': 100, 'min_mouvement': 0.25, 'activate_vision_buoys': True}),
										transitions={'finished': 'move_2', 'failed': 'failed', 'lost_target': 'move_2'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:200 y:714
			OperatableStateMachine.add('vision_droppers',
										self.use_behavior(vision_droppersSM, 'vision_droppers',
											parameters={'vision_droppers_target': "Notepad", 'bounding_box_height': 90, 'bounding_box_width': 115, 'center_bounding_box_height': 50, 'center_bounding_box_width': 50, 'max_mouvement': 0.5, 'min_mouvement': 0.1, 'activate_vision_droppers': True}),
										transitions={'finished': 'drop_AUV7', 'failed': 'failed', 'lost_target': 'vision_droppers_2', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:696 y:600
			OperatableStateMachine.add('vision_droppers_2',
										self.use_behavior(vision_droppersSM, 'vision_droppers_2',
											parameters={'vision_droppers_target': "Phone", 'bounding_box_height': 90, 'bounding_box_width': 115, 'center_bounding_box_height': 50, 'center_bounding_box_width': 50, 'max_mouvement': 0.5, 'min_mouvement': 0.1, 'activate_vision_droppers': True}),
										transitions={'finished': 'drop_AUV7', 'failed': 'failed', 'lost_target': 'move', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:228 y:134
			OperatableStateMachine.add('vision_path',
										self.use_behavior(vision_pathSM, 'vision_path',
											parameters={'vision_path_target': "pipe straight", 'min_mouvement': 0.25, 'max_mouvement': 1.2, 'bounding_box_height': 100, 'bounding_box_width': 30, 'center_bounding_box_height': 150, 'center_bounding_box_width': 150, 'activate_vision_path': True, 'move_after_path': 0}),
										transitions={'finished': 'move_with_detection_buoys', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'angle': 'angle', 'camera': 'camera'})

			# x:204 y:481
			OperatableStateMachine.add('vision_path_2',
										self.use_behavior(vision_pathSM, 'vision_path_2',
											parameters={'vision_path_target': "pipe straight", 'min_mouvement': 0.25, 'max_mouvement': 1.2, 'bounding_box_height': 100, 'bounding_box_width': 30, 'center_bounding_box_height': 150, 'center_bounding_box_width': 150, 'move_after_path': 6}),
										transitions={'finished': 'vision_bins', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'angle': 'angle', 'camera': 'camera'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
