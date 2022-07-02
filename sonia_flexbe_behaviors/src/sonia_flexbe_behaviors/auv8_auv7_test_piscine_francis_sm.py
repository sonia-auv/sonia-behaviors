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
from sonia_com_states.get_update import get_update
from sonia_com_states.send_update import send_update
from sonia_com_states.synchro_receive import synchro_receive
from sonia_com_states.synchro_send import synchro_send
from sonia_flexbe_behaviors.coin_flip_ky_sm import coin_flip_KYSM
from sonia_flexbe_behaviors.init_submarine_sm import init_submarineSM
from sonia_flexbe_behaviors.move_to_gate_straight_sm import Move_to_gate_straightSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jiuil 02 2022
@author: KY
'''
class AUV8_AUV7_Test_piscine_francisSM(Behavior):
	'''
	Run coinflip, forward and intercom
	'''


	def __init__(self):
		super(AUV8_AUV7_Test_piscine_francisSM, self).__init__()
		self.name = 'AUV8_AUV7_Test_piscine_francis'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(Move_to_gate_straightSM, 'Move_to_gate_straight_AUV7')
		self.add_behavior(coin_flip_KYSM, 'coin_flip_KY')
		self.add_behavior(init_submarineSM, 'init_submarine')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1143 y:356, x:708 y:252
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:31 y:72
			OperatableStateMachine.add('init_submarine',
										self.use_behavior(init_submarineSM, 'init_submarine'),
										transitions={'finished': 'Choose_sub', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:686 y:112
			OperatableStateMachine.add('AUV7_gate_failed',
										send_update(mission=1, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:43 y:161
			OperatableStateMachine.add('Choose_sub',
										choose_your_character(submarine='AUV8'),
										transitions={'auv8': 'coin_flip_KY', 'auv7': 'waiting_for_friend'},
										autonomy={'auv8': Autonomy.Off, 'auv7': Autonomy.Off})

			# x:29 y:345
			OperatableStateMachine.add('Coin_flip_completed',
										send_update(mission=0, state=2),
										transitions={'continue': 'Move_now'},
										autonomy={'continue': Autonomy.Off})

			# x:240 y:246
			OperatableStateMachine.add('Coin_flip_failed',
										send_update(mission=0, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:694 y:354
			OperatableStateMachine.add('Get_update_of_AUV7',
										get_update(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'mission_array': 'mission_array'})

			# x:454 y:343
			OperatableStateMachine.add('Move_now',
										synchro_send(),
										transitions={'continue': 'Get_update_of_AUV7'},
										autonomy={'continue': Autonomy.Off})

			# x:481 y:43
			OperatableStateMachine.add('Move_to_gate_straight_AUV7',
										self.use_behavior(Move_to_gate_straightSM, 'Move_to_gate_straight_AUV7'),
										transitions={'finished': 'AUV7_gate_completed', 'failed': 'AUV7_gate_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:29 y:247
			OperatableStateMachine.add('coin_flip_KY',
										self.use_behavior(coin_flip_KYSM, 'coin_flip_KY'),
										transitions={'finished': 'Coin_flip_completed', 'failed': 'Coin_flip_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1095 y:152
			OperatableStateMachine.add('get_update_AUV8',
										get_update(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'mission_array': 'mission_array'})

			# x:249 y:160
			OperatableStateMachine.add('waiting_for_friend',
										synchro_receive(timeout=120),
										transitions={'continue': 'Move_to_gate_straight_AUV7', 'timeout': 'Move_to_gate_straight_AUV7'},
										autonomy={'continue': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:889 y:65
			OperatableStateMachine.add('AUV7_gate_completed',
										send_update(mission=1, state=2),
										transitions={'continue': 'get_update_AUV8'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
