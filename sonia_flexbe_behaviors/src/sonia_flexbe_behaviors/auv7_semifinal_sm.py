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
from sonia_flexbe_behaviors.square_sm import squareSM
from sonia_flexbe_behaviors.vision_bins_sm import vision_binsSM
from sonia_flexbe_behaviors.vision_buoys_sm import vision_buoysSM
from sonia_flexbe_behaviors.vision_droppers_sm import vision_droppersSM
from sonia_flexbe_behaviors.vision_path_sm import vision_pathSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 30 2022
@author: WK
'''
class AUV7_SEMIFINALSM(Behavior):
	'''
	AUV7_SEMIFINAL
	'''


	def __init__(self):
		super(AUV7_SEMIFINALSM, self).__init__()
		self.name = 'AUV7_SEMIFINAL'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com')
		self.add_behavior(drop_AUV7SM, 'drop_AUV7')
		self.add_behavior(squareSM, 'square')
		self.add_behavior(vision_binsSM, 'vision_bins')
		self.add_behavior(vision_buoysSM, 'vision_buoys')
		self.add_behavior(vision_droppersSM, 'vision_droppers')
		self.add_behavior(vision_droppersSM, 'vision_droppers_2')
		self.add_behavior(vision_pathSM, 'vision_path')
		self.add_behavior(vision_pathSM, 'vision_path_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1631 y:475, x:1163 y:28
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:178 y:51
			OperatableStateMachine.add('CoinFlip-Gate-Trickshot with com',
										self.use_behavior(CoinFlipGateTrickshotwithcomSM, 'CoinFlip-Gate-Trickshot with com'),
										transitions={'finished': 'vision_path', 'failed': 'failed', 'failed_start_control': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'failed_start_control': Autonomy.Inherit})

			# x:1543 y:313
			OperatableStateMachine.add('drop_AUV7',
										self.use_behavior(drop_AUV7SM, 'drop_AUV7'),
										transitions={'finished': 'square', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1373 y:475
			OperatableStateMachine.add('square',
										self.use_behavior(squareSM, 'square'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:298 y:399
			OperatableStateMachine.add('vision_bins',
										self.use_behavior(vision_binsSM, 'vision_bins'),
										transitions={'finished': 'vision_droppers', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:235 y:225
			OperatableStateMachine.add('vision_buoys',
										self.use_behavior(vision_buoysSM, 'vision_buoys'),
										transitions={'finished': 'vision_path_2', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:329 y:548
			OperatableStateMachine.add('vision_droppers',
										self.use_behavior(vision_droppersSM, 'vision_droppers'),
										transitions={'finished': 'drop_AUV7', 'failed': 'failed', 'lost_target': 'vision_droppers_2', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:946 y:305
			OperatableStateMachine.add('vision_droppers_2',
										self.use_behavior(vision_droppersSM, 'vision_droppers_2'),
										transitions={'finished': 'drop_AUV7', 'failed': 'failed', 'lost_target': 'failed', 'controller_error': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit, 'controller_error': Autonomy.Inherit})

			# x:31 y:165
			OperatableStateMachine.add('vision_path',
										self.use_behavior(vision_pathSM, 'vision_path'),
										transitions={'finished': 'vision_buoys', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'angle': 'angle', 'camera': 'camera'})

			# x:479 y:261
			OperatableStateMachine.add('vision_path_2',
										self.use_behavior(vision_pathSM, 'vision_path_2'),
										transitions={'finished': 'vision_bins', 'failed': 'failed', 'lost_target': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit},
										remapping={'angle': 'angle', 'camera': 'camera'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
