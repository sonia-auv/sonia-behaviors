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
from sonia_flexbe_behaviors.coinflip_with_com_sm import CoinFlipwithcomSM
from sonia_flexbe_behaviors.gate_with_com_sm import GatewithcomSM
from sonia_flexbe_behaviors.init_submarine_with_com_sm import init_submarine_with_comSM
from sonia_flexbe_behaviors.trickshot_with_com_sm import TrickshotwithcomSM
from sonia_flexbe_states.activate_behavior import activate_behavior
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
		self.add_parameter('distance_to_gate', 3.5)
		self.add_parameter('dive_depth', 1.5)
		self.add_parameter('has_com', True)
		self.add_parameter('activate_coinflip_gate_trickshot_com', True)

		# references to used behaviors
		self.add_behavior(CoinFlipwithcomSM, 'CoinFlip with com')
		self.add_behavior(GatewithcomSM, 'Gate with com')
		self.add_behavior(TrickshotwithcomSM, 'Trickshot with com (takeover)')
		self.add_behavior(init_submarine_with_comSM, 'init_submarine_with_com')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:951 y:333, x:513 y:177, x:361 y:142
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'failed_start_control'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:65 y:40
			OperatableStateMachine.add('activate',
										activate_behavior(activate=self.activate_coinflip_gate_trickshot_com),
										transitions={'activate': 'init_submarine_with_com', 'desactivate': 'finished'},
										autonomy={'activate': Autonomy.Off, 'desactivate': Autonomy.Off})

			# x:80 y:205
			OperatableStateMachine.add('CoinFlip with com',
										self.use_behavior(CoinFlipwithcomSM, 'CoinFlip with com',
											parameters={'dive_depth': self.dive_depth}),
										transitions={'finished': 'Choose_sub', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:425 y:399
			OperatableStateMachine.add('Gate with com',
										self.use_behavior(GatewithcomSM, 'Gate with com',
											parameters={'distance_to_gate': self.distance_to_gate}),
										transitions={'finished': 'taking_over_trickshot', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1138 y:299
			OperatableStateMachine.add('Move_now',
										synchro_send(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:768 y:70
			OperatableStateMachine.add('Trickshot with com (takeover)',
										self.use_behavior(TrickshotwithcomSM, 'Trickshot with com (takeover)',
											parameters={'has_com': self.has_com}),
										transitions={'finished': 'choose_sub_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1098 y:67
			OperatableStateMachine.add('choose_sub_2',
										choose_your_character(submarine=self.submarine),
										transitions={'auv8': 'Move_now', 'auv7': 'finished'},
										autonomy={'auv8': Autonomy.Off, 'auv7': Autonomy.Off})

			# x:67 y:118
			OperatableStateMachine.add('init_submarine_with_com',
										self.use_behavior(init_submarine_with_comSM, 'init_submarine_with_com',
											parameters={'sub_init_array': "1,1,1,0,0,0,0,1,1,1,1", 'other_sub_init_array': "1,1,0,1,1,1,1,0,0,0,0", 'submarine': self.submarine}),
										transitions={'finished': 'CoinFlip with com', 'failed_start_control': 'failed_start_control'},
										autonomy={'finished': Autonomy.Inherit, 'failed_start_control': Autonomy.Inherit})

			# x:685 y:382
			OperatableStateMachine.add('taking_over_trickshot',
										takeover_mission(mission_id=2, has_com=self.has_com),
										transitions={'takeover': 'Trickshot with com (takeover)', 'no_takeover': 'finished', 'assigned': 'Trickshot with com (takeover)'},
										autonomy={'takeover': Autonomy.Off, 'no_takeover': Autonomy.Off, 'assigned': Autonomy.Off})

			# x:204 y:421
			OperatableStateMachine.add('waiting_for_friend',
										synchro_receive(has_com=self.has_com, timeout=45),
										transitions={'continue': 'Gate with com', 'timeout': 'Gate with com'},
										autonomy={'continue': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:134 y:309
			OperatableStateMachine.add('Choose_sub',
										choose_your_character(submarine=self.submarine),
										transitions={'auv8': 'Gate with com', 'auv7': 'waiting_for_friend'},
										autonomy={'auv8': Autonomy.Off, 'auv7': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
