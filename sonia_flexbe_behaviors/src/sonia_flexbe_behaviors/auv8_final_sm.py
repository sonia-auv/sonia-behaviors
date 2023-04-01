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
from sonia_flexbe_behaviors.coinflip_gate_trickshot_with_com_sm import CoinFlipGateTrickshotwithcomSM
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_flexbe_behaviors.move_with_detection_torpedoes_board_sm import move_with_detection_torpedoes_boardSM
from sonia_flexbe_behaviors.search_zigzag_sm import search_zigzagSM
from sonia_flexbe_behaviors.sonar_table_alignement_sm import sonar_table_alignementSM
from sonia_flexbe_behaviors.torpedoes_sm import torpedoesSM
from sonia_vision_states.start_filter_chain import start_filter_chain
from sonia_vision_states.stop_filter_chain import stop_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 30 2022
@author: GS
'''
class AUV8_FINALSM(Behavior):
	'''
	Full run for AUV8
	'''


	def __init__(self):
		super(AUV8_FINALSM, self).__init__()
		self.name = 'AUV8_FINAL'

		# parameters of this behavior
		self.add_parameter('camera', 1)
		self.add_parameter('backup_filterchain', 'deep_compe_front')

		# references to used behaviors
		self.add_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com')
		self.add_behavior(moveSM, 'move')
		self.add_behavior(moveSM, 'move_2')
		self.add_behavior(move_with_detection_torpedoes_boardSM, 'move_with_detection_torpedoes_board')
		self.add_behavior(search_zigzagSM, 'search_zigzag')
		self.add_behavior(sonar_table_alignementSM, 'sonar_table_alignement')
		self.add_behavior(torpedoesSM, 'torpedoes')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1174 y:617, x:594 y:331
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('CoinFlip-Gate-Trickshot with com',
										self.use_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com',
											parameters={'submarine': "AUV8", 'distance_to_gate': 6, 'dive_depth': 1.5, 'has_com': True}),
										transitions={'finished': 'move_with_detection_torpedoes_board', 'failed': 'failed', 'failed_start_control': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'failed_start_control': Autonomy.Inherit})

			# x:35 y:478
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': 0, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 2, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'move_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:52 y:251
			OperatableStateMachine.add('move_2',
										self.use_behavior(moveSM, 'move_2',
											parameters={'positionX': -2, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'move_with_detection_torpedoes_board', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:420 y:44
			OperatableStateMachine.add('move_with_detection_torpedoes_board',
										self.use_behavior(move_with_detection_torpedoes_boardSM, 'move_with_detection_torpedoes_board',
											parameters={'move_for_tb': 9, 'turn_for_tb': 0}),
										transitions={'finished': 'torpedoes', 'failed': 'failed', 'lost_target': 'start_filter_backup'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:1136 y:455
			OperatableStateMachine.add('octogno_sucess',
										send_update(mission=8, state=2),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:844 y:349
			OperatableStateMachine.add('octogon_failed',
										send_update(mission=8, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:546 y:655
			OperatableStateMachine.add('search_zigzag',
										self.use_behavior(search_zigzagSM, 'search_zigzag'),
										transitions={'finished': 'stop_filter_backup', 'failed': 'failed', 'lost_target': 'move', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit},
										remapping={'target': 'target', 'topic': 'topic'})

			# x:1101 y:349
			OperatableStateMachine.add('sonar_table_alignement',
										self.use_behavior(sonar_table_alignementSM, 'sonar_table_alignement'),
										transitions={'finished': 'octogno_sucess', 'failed': 'octogon_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:441 y:495
			OperatableStateMachine.add('start_filter_backup',
										start_filter_chain(filterchain=self.backup_filterchain, target="G-Man", camera_no=self.camera),
										transitions={'continue': 'search_zigzag', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:872 y:567
			OperatableStateMachine.add('stop_filter_backup',
										stop_filter_chain(),
										transitions={'continue': 'torpedoes', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:1068 y:91
			OperatableStateMachine.add('torpedoes',
										self.use_behavior(torpedoesSM, 'torpedoes'),
										transitions={'finished': 'torpedos_sucess', 'failed': 'torpedos_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:828 y:200
			OperatableStateMachine.add('torpedos_failed',
										send_update(mission=7, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:1116 y:216
			OperatableStateMachine.add('torpedos_sucess',
										send_update(mission=7, state=2),
										transitions={'continue': 'sonar_table_alignement'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
