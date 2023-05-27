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
from sonia_flexbe_behaviors.sonar_table_alignement_sm import sonar_table_alignementSM
from sonia_flexbe_behaviors.torpedoes_sonar_sm import torpedoes_sonarSM
from sonia_flexbe_behaviors.vision_torpedoes_boards_sm import vision_torpedoes_boardsSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 30 2022
@author: GS
'''
class AUV8_SEMIFINAL_2_SONARSM(Behavior):
	'''
	Full run for AUV8 semi finale 2
	'''


	def __init__(self):
		super(AUV8_SEMIFINAL_2_SONARSM, self).__init__()
		self.name = 'AUV8_SEMIFINAL_2_SONAR'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com')
		self.add_behavior(moveSM, 'move')
		self.add_behavior(sonar_table_alignementSM, 'sonar_table_alignement')
		self.add_behavior(torpedoes_sonarSM, 'torpedoes_sonar')
		self.add_behavior(vision_torpedoes_boardsSM, 'vision_torpedoes_boards')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1251 y:305, x:584 y:303
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:48 y:47
			OperatableStateMachine.add('CoinFlip-Gate-Trickshot with com',
										self.use_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com',
											parameters={'submarine': "AUV8", 'dive_depth': 1.5, 'has_com': True}),
										transitions={'finished': 'move', 'failed': 'failed', 'failed_start_control': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'failed_start_control': Autonomy.Inherit})

			# x:404 y:48
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'vision_torpedoes_boards', 'failed': 'vision_torpedoes_boards'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:990 y:290
			OperatableStateMachine.add('sonar_table_alignement',
										self.use_behavior(sonar_table_alignementSM, 'sonar_table_alignement'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:984 y:47
			OperatableStateMachine.add('torpedoes_sonar',
										self.use_behavior(torpedoes_sonarSM, 'torpedoes_sonar'),
										transitions={'finished': 'sonar_table_alignement', 'failed': 'sonar_table_alignement'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:676 y:47
			OperatableStateMachine.add('vision_torpedoes_boards',
										self.use_behavior(vision_torpedoes_boardsSM, 'vision_torpedoes_boards',
											parameters={'vision_torpedoes_boards_target': "G-Man", 'center_bounding_box_width': 100, 'center_bounding_box_height': 100, 'max_mouvement': 2, 'min_mouvement': 0.25, 'activate_vision_buoys': True}),
										transitions={'finished': 'torpedoes_sonar', 'failed': 'failed', 'lost_target': 'sonar_table_alignement'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
