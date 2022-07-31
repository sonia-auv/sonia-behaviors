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
from sonia_flexbe_behaviors.torpedoes_sm import torpedoesSM
from sonia_flexbe_behaviors.vision_buoys_auv8_sm import vision_buoys_AUV8SM
from sonia_flexbe_behaviors.vision_path_sm import vision_pathSM
from sonia_flexbe_behaviors.vision_tables_sm import vision_tablesSM
from sonia_flexbe_behaviors.vision_torpedoes_boards_sm import vision_torpedoes_boardsSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 30 2022
@author: GS
'''
class AUV8_SEMIFINAL_FULL_TASKSSM(Behavior):
	'''
	Full run for AUV8 with tasks of AUV7
	'''


	def __init__(self):
		super(AUV8_SEMIFINAL_FULL_TASKSSM, self).__init__()
		self.name = 'AUV8_SEMIFINAL_FULL_TASKS'

		# parameters of this behavior
		self.add_parameter('angle_after_path', '10')

		# references to used behaviors
		self.add_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com')
		self.add_behavior(torpedoesSM, 'torpedoes')
		self.add_behavior(vision_buoys_AUV8SM, 'vision_buoys_AUV8')
		self.add_behavior(vision_pathSM, 'vision_path_2')
		self.add_behavior(vision_tablesSM, 'vision_tables')
		self.add_behavior(vision_torpedoes_boardsSM, 'vision_torpedoes_boards')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1186 y:451, x:566 y:297
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:54 y:38
			OperatableStateMachine.add('CoinFlip-Gate-Trickshot with com',
										self.use_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com'),
										transitions={'finished': 'vision_path_2', 'failed': 'failed', 'failed_start_control': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'failed_start_control': Autonomy.Inherit})

			# x:599 y:431
			OperatableStateMachine.add('torpedoes',
										self.use_behavior(torpedoesSM, 'torpedoes'),
										transitions={'finished': 'vision_tables', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:69 y:301
			OperatableStateMachine.add('vision_buoys_AUV8',
										self.use_behavior(vision_buoys_AUV8SM, 'vision_buoys_AUV8'),
										transitions={'finished': 'vision_torpedoes_boards', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:67 y:181
			OperatableStateMachine.add('vision_path_2',
										self.use_behavior(vision_pathSM, 'vision_path_2'),
										transitions={'finished': 'vision_buoys_AUV8', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'angle': 'angle', 'camera': 'camera'})

			# x:913 y:434
			OperatableStateMachine.add('vision_tables',
										self.use_behavior(vision_tablesSM, 'vision_tables'),
										transitions={'finished': 'finished', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:215 y:431
			OperatableStateMachine.add('vision_torpedoes_boards',
										self.use_behavior(vision_torpedoes_boardsSM, 'vision_torpedoes_boards'),
										transitions={'finished': 'torpedoes', 'failed': 'failed', 'lost_target': 'torpedoes'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
