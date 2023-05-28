#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_com_states.send_update import send_update
from sonia_flexbe_behaviors.avoid_buoys_after_collision_sm import avoid_buoys_after_collisionSM
from sonia_flexbe_behaviors.coinflip_gate_trickshot_with_com_sm import CoinFlipGateTrickshotwithcomSM
from sonia_flexbe_behaviors.drop_auv7_sm import drop_AUV7SM
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_flexbe_behaviors.move_with_detection_buoys_sm import move_with_detection_buoysSM
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
		self.add_behavior(avoid_buoys_after_collisionSM, 'avoid_buoys_after_collision')
		self.add_behavior(drop_AUV7SM, 'drop_AUV7')
		self.add_behavior(moveSM, 'move')
		self.add_behavior(moveSM, 'move_2')
		self.add_behavior(moveSM, 'move_3')
		self.add_behavior(move_with_detection_buoysSM, 'move_with_detection_buoys')
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
		# x:1100 y:61, x:604 y:350
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:212 y:17
			OperatableStateMachine.add('CoinFlip-Gate-Trickshot with com',
										self.use_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com',
											parameters={'submarine': "AUV7", 'activate_coinflip_gate_trickshot_com': True}),
										transitions={'finished': 'vision_path', 'failed': 'failed', 'failed_start_control': 'path1_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'failed_start_control': Autonomy.Inherit})

			# x:77 y:489
			OperatableStateMachine.add('avoid_buoys_after_collision',
										self.use_behavior(avoid_buoys_after_collisionSM, 'avoid_buoys_after_collision',
											parameters={'distance_up_after_collision': 1, 'distance_forward_after_collision': 2.0, 'distance_down_after_collision': 1, 'activate_avoid_buoys_after_collision': True}),
										transitions={'finished': 'move_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:392 y:341
			OperatableStateMachine.add('buoy_failed',
										send_update(mission=4, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:17 y:374
			OperatableStateMachine.add('buoy_sucess',
										send_update(mission=4, state=2),
										transitions={'continue': 'avoid_buoys_after_collision'},
										autonomy={'continue': Autonomy.Off})

			# x:1021 y:193
			OperatableStateMachine.add('drop_AUV7',
										self.use_behavior(drop_AUV7SM, 'drop_AUV7',
											parameters={'activate_drop_auv7': True}),
										transitions={'finished': 'drop_success', 'failed': 'drop_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:838 y:217
			OperatableStateMachine.add('drop_failed',
										send_update(mission=6, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:1051 y:110
			OperatableStateMachine.add('drop_success',
										send_update(mission=6, state=2),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:856 y:760
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': 10, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'vision_bins', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:104 y:620
			OperatableStateMachine.add('move_2',
										self.use_behavior(moveSM, 'move_2',
											parameters={'positionX': 0, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 180, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'vision_path_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:48 y:692
			OperatableStateMachine.add('move_3',
										self.use_behavior(moveSM, 'move_3',
											parameters={'positionX': 3}),
										transitions={'finished': 'buoy_sucess', 'failed': 'buoy_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:194 y:251
			OperatableStateMachine.add('move_with_detection_buoys',
										self.use_behavior(move_with_detection_buoysSM, 'move_with_detection_buoys',
											parameters={'move_for_buoys': 10, 'buoys_filterchain': "deep_compe_front", 'buoys_target': "Badge"}),
										transitions={'finished': 'vision_buoys_collision_detector', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:547 y:37
			OperatableStateMachine.add('path1_failed',
										send_update(mission=3, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:25 y:200
			OperatableStateMachine.add('path1_sucess',
										send_update(mission=3, state=2),
										transitions={'continue': 'move_with_detection_buoys'},
										autonomy={'continue': Autonomy.Off})

			# x:551 y:532
			OperatableStateMachine.add('path2_failed',
										send_update(mission=5, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:590 y:786
			OperatableStateMachine.add('path2_sucess',
										send_update(mission=5, state=2),
										transitions={'continue': 'move'},
										autonomy={'continue': Autonomy.Off})

			# x:1076 y:647
			OperatableStateMachine.add('vision_bins',
										self.use_behavior(vision_binsSM, 'vision_bins',
											parameters={'vision_bins_filterchain': "simple_bin", 'vision_bins_target': "cover", 'camera_no': 2, 'bounding_box_width': 200, 'bounding_box_height': 200, 'center_bb_height': 50, 'center_bb_width': 50, 'max_mouvement': 1.0, 'min_mouvement': 0.1, 'activate_vision_bins': True}),
										transitions={'finished': 'vision_droppers', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:177 y:345
			OperatableStateMachine.add('vision_buoys_collision_detector',
										self.use_behavior(vision_buoys_collision_detectorSM, 'vision_buoys_collision_detector',
											parameters={'vision_buoys_filterchain': "deep_compe_front", 'camera_no': 1, 'vision_buoys_target': "Badge", 'bounding_box_width': 100, 'bounding_box_height': 100, 'center_bounding_box_width': 100, 'center_bounding_box_height': 100, 'min_mouvement': 0.25, 'activate_vision_buoys': True}),
										transitions={'finished': 'move_3', 'failed': 'buoy_failed', 'lost_target': 'move_3'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:991 y:526
			OperatableStateMachine.add('vision_droppers',
										self.use_behavior(vision_droppersSM, 'vision_droppers',
											parameters={'vision_droppers_filterchain': "deep_compe_bottom", 'camera_no': 2, 'bounding_box_height': 90, 'bounding_box_width': 115, 'center_bounding_box_height': 50, 'center_bounding_box_width': 50, 'max_mouvement': 0.5, 'min_mouvement': 0.1, 'activate_vision_droppers': True}),
										transitions={'finished': 'drop_AUV7', 'failed': 'failed', 'lost_target': 'vision_droppers_2', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:867 y:366
			OperatableStateMachine.add('vision_droppers_2',
										self.use_behavior(vision_droppersSM, 'vision_droppers_2',
											parameters={'vision_droppers_filterchain': "deep_compe_bottom", 'camera_no': 2, 'bounding_box_height': 90, 'bounding_box_width': 115, 'center_bounding_box_height': 50, 'center_bounding_box_width': 50, 'max_mouvement': 0.5, 'min_mouvement': 0.1, 'activate_vision_droppers': True}),
										transitions={'finished': 'drop_AUV7', 'failed': 'failed', 'lost_target': 'drop_AUV7', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:169 y:125
			OperatableStateMachine.add('vision_path',
										self.use_behavior(vision_pathSM, 'vision_path',
											parameters={'vision_path_filterchain': "simple_pipe_straight", 'vision_path_target': "pipe straight", 'camera_no': 2, 'min_mouvement': 0.25, 'max_mouvement': 1.2, 'bounding_box_height': 100, 'bounding_box_width': 30, 'center_bounding_box_height': 150, 'center_bounding_box_width': 150, 'activate_vision_path': True}),
										transitions={'finished': 'path1_sucess', 'failed': 'path1_failed', 'lost_target': 'path1_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'angle': 'angle', 'camera': 'camera'})

			# x:376 y:715
			OperatableStateMachine.add('vision_path_2',
										self.use_behavior(vision_pathSM, 'vision_path_2',
											parameters={'vision_path_filterchain': "simple_pipe_straight", 'vision_path_target': "pipe straight", 'camera_no': 2, 'min_mouvement': 0.25, 'max_mouvement': 1.2, 'bounding_box_height': 100, 'bounding_box_width': 30, 'center_bounding_box_height': 150, 'center_bounding_box_width': 150}),
										transitions={'finished': 'path2_sucess', 'failed': 'path2_failed', 'lost_target': 'path2_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'angle': 'angle', 'camera': 'camera'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
