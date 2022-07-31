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
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_flexbe_behaviors.torpedoes_sm import torpedoesSM
from sonia_flexbe_behaviors.vision_tables_sm import vision_tablesSM
from sonia_flexbe_behaviors.vision_torpedoes_boards_sm import vision_torpedoes_boardsSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 30 2022
@author: GS
'''
class AUV8_SEMIFINALSM(Behavior):
	'''
	Full run for AUV8
	'''


	def __init__(self):
		super(AUV8_SEMIFINALSM, self).__init__()
		self.name = 'AUV8_SEMIFINAL'

		# parameters of this behavior
		self.add_parameter('angle_after_path', 10)

		# references to used behaviors
		self.add_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com')
		self.add_behavior(moveSM, 'move')
		self.add_behavior(torpedoesSM, 'torpedoes')
		self.add_behavior(vision_tablesSM, 'vision_tables')
		self.add_behavior(vision_torpedoes_boardsSM, 'vision_torpedoes_boards')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1186 y:513, x:566 y:297
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('CoinFlip-Gate-Trickshot with com',
										self.use_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com'),
										transitions={'finished': 'move', 'failed': 'failed', 'failed_start_control': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'failed_start_control': Autonomy.Inherit})

			# x:376 y:48
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': self.angle_after_path, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'vision_torpedoes_boards', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:994 y:65
			OperatableStateMachine.add('torpedoes',
										self.use_behavior(torpedoesSM, 'torpedoes'),
										transitions={'finished': 'vision_tables', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1110 y:383
			OperatableStateMachine.add('vision_tables',
										self.use_behavior(vision_tablesSM, 'vision_tables'),
										transitions={'finished': 'finished', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:638 y:49
			OperatableStateMachine.add('vision_torpedoes_boards',
										self.use_behavior(vision_torpedoes_boardsSM, 'vision_torpedoes_boards'),
										transitions={'finished': 'torpedoes', 'failed': 'failed', 'lost_target': 'torpedoes'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
