#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_com_states.choose_your_character import choose_your_character
from sonia_com_states.synchro_receive import synchro_receive
from sonia_com_states.synchro_send import synchro_send
from sonia_com_states.takeover_mission import takeover_mission
from sonia_flexbe_behaviors.coinflip_gate_with_com_sm import CoinFlipGatewithcomSM
from sonia_flexbe_behaviors.coinflip_with_com_sm import CoinFlipwithcomSM
from sonia_flexbe_behaviors.gate_with_com_sm import GatewithcomSM
from sonia_flexbe_behaviors.init_submarine_with_com_sm import init_submarine_with_comSM
from sonia_flexbe_behaviors.trickshot_with_com_sm import TrickshotwithcomSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 17 2022
@author: FA
'''
class CoinFlipGateTrickshotwithcomSM(Behavior):
	'''
	With the behavior on both submarines, this behavior regroup coin-flip, gate and trickshot. Include the communication
	'''


	def __init__(self):
		super(CoinFlipGateTrickshotwithcomSM, self).__init__()
		self.name = 'CoinFlip-Gate-Trickshot with com'

		# parameters of this behavior
		self.add_parameter('submarine', 'AUV8')
		self.add_parameter('distance_to_gate', 4)
		self.add_parameter('dive_depth', 1)

		# references to used behaviors
		self.add_behavior(CoinFlipwithcomSM, 'CoinFlip with com')
		self.add_behavior(CoinFlipGatewithcomSM, 'CoinFlip-Gate with com')
		self.add_behavior(GatewithcomSM, 'Gate with com')
		self.add_behavior(TrickshotwithcomSM, 'Trickshot with com')
		self.add_behavior(TrickshotwithcomSM, 'Trickshot with com (takeover)')
		self.add_behavior(init_submarine_with_comSM, 'init_submarine_with_com')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1061 y:511, x:620 y:293, x:450 y:48
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'failed_start_control'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:72 y:40
			OperatableStateMachine.add('init_submarine_with_com',
										self.use_behavior(init_submarine_with_comSM, 'init_submarine_with_com',
											parameters={'simulation': False, 'sub_init_array': "1,1,1,0,0,0,0,1,1,1,1", 'other_sub_init_array': "1,1,0,1,1,1,1,0,0,0,0", 'submarine': self.submarine}),
										transitions={'finished': 'Choose_sub', 'failed_start_control': 'failed_start_control'},
										autonomy={'finished': Autonomy.Inherit, 'failed_start_control': Autonomy.Inherit})

			# x:170 y:263
			OperatableStateMachine.add('CoinFlip with com',
										self.use_behavior(CoinFlipwithcomSM, 'CoinFlip with com',
											parameters={'orientation_to_gate': 0, 'dive_depth': self.dive_depth}),
										transitions={'finished': 'waiting_for_friend', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:299 y:139
			OperatableStateMachine.add('CoinFlip-Gate with com',
										self.use_behavior(CoinFlipGatewithcomSM, 'CoinFlip-Gate with com',
											parameters={'orientation_to_gate': 0, 'dive_depth': self.dive_depth, 'distance_to_gate': self.distance_to_gate}),
										transitions={'finished': 'Trickshot with com', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:67 y:538
			OperatableStateMachine.add('Gate with com',
										self.use_behavior(GatewithcomSM, 'Gate with com',
											parameters={'distance_to_gate': self.distance_to_gate}),
										transitions={'finished': 'taking_over_trickshot', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1012 y:49
			OperatableStateMachine.add('Move_now',
										synchro_send(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:661 y:44
			OperatableStateMachine.add('Trickshot with com',
										self.use_behavior(TrickshotwithcomSM, 'Trickshot with com'),
										transitions={'finished': 'Move_now'},
										autonomy={'finished': Autonomy.Inherit})

			# x:585 y:413
			OperatableStateMachine.add('Trickshot with com (takeover)',
										self.use_behavior(TrickshotwithcomSM, 'Trickshot with com (takeover)'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:463 y:607
			OperatableStateMachine.add('taking_over_trickshot',
										takeover_mission(mission_id=2),
										transitions={'takeover': 'Trickshot with com (takeover)', 'no_takeover': 'finished', 'assigned': 'Trickshot with com (takeover)'},
										autonomy={'takeover': Autonomy.Off, 'no_takeover': Autonomy.Off, 'assigned': Autonomy.Off})

			# x:76 y:381
			OperatableStateMachine.add('waiting_for_friend',
										synchro_receive(timeout=45),
										transitions={'continue': 'Gate with com', 'timeout': 'Gate with com'},
										autonomy={'continue': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:66 y:151
			OperatableStateMachine.add('Choose_sub',
										choose_your_character(submarine='AUV8'),
										transitions={'auv8': 'CoinFlip-Gate with com', 'auv7': 'CoinFlip with com'},
										autonomy={'auv8': Autonomy.Off, 'auv7': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
