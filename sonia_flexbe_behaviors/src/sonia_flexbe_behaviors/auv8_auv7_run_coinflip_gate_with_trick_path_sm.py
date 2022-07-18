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
from sonia_com_states.takeover_mission import takeover_mission
from sonia_flexbe_behaviors.coin_flip_ky_sm import coin_flip_KYSM
from sonia_flexbe_behaviors.init_submarine_sm import init_submarineSM
from sonia_flexbe_behaviors.move_to_gate_straight_sm import Move_to_gate_straightSM
from sonia_flexbe_behaviors.trickshot_sm import trickshotSM
from sonia_flexbe_behaviors.vision_path_new_algo_sm import vision_path_new_algoSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jiuil 02 2022
@author: KY
'''
class AUV8_AUV7_Run_coinflip_gate_with_trick_pathSM(Behavior):
	'''
	Run coinflip, gate without trickshot and path with intercom
	'''


	def __init__(self):
		super(AUV8_AUV7_Run_coinflip_gate_with_trick_pathSM, self).__init__()
		self.name = 'AUV8_AUV7_Run_coinflip_gate_with_trick_path'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(Move_to_gate_straightSM, 'Move_to_gate_straight_3')
		self.add_behavior(Move_to_gate_straightSM, 'Move_to_gate_straight_AUV7')
		self.add_behavior(Move_to_gate_straightSM, 'Move_to_gate_straight_AUV8')
		self.add_behavior(vision_path_new_algoSM, 'Path')
		self.add_behavior(coin_flip_KYSM, 'coin_flip_KY')
		self.add_behavior(init_submarineSM, 'init_submarine')
		self.add_behavior(trickshotSM, 'trickshot')
		self.add_behavior(trickshotSM, 'trickshot_if_not_done')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1246 y:391, x:796 y:465
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:31 y:72
			OperatableStateMachine.add('init_submarine',
										self.use_behavior(init_submarineSM, 'init_submarine'),
										transitions={'finished': 'coin_flip_KY', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:478 y:415
			OperatableStateMachine.add('AUV7_gate_failed',
										send_update(mission=1, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:785 y:662
			OperatableStateMachine.add('AUV7_path_completed',
										send_update(mission=3, state=2),
										transitions={'continue': 'get_update_AUV8'},
										autonomy={'continue': Autonomy.Off})

			# x:751 y:546
			OperatableStateMachine.add('AUV7_path_failed',
										send_update(mission=3, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:364 y:585
			OperatableStateMachine.add('AUV7_trickshot_completed',
										send_update(mission=2, state=2),
										transitions={'continue': 'Path'},
										autonomy={'continue': Autonomy.Off})

			# x:503 y:499
			OperatableStateMachine.add('AUV7_trickshot_failed',
										send_update(mission=2, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:470 y:115
			OperatableStateMachine.add('AUV8_gate_completed',
										send_update(mission=1, state=2),
										transitions={'continue': 'AUV8_trickshot_failed'},
										autonomy={'continue': Autonomy.Off})

			# x:509 y:205
			OperatableStateMachine.add('AUV8_gate_failed',
										send_update(mission=1, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:878 y:111
			OperatableStateMachine.add('AUV8_trickshot_completed',
										send_update(mission=2, state=2),
										transitions={'continue': 'Move_now'},
										autonomy={'continue': Autonomy.Off})

			# x:704 y:217
			OperatableStateMachine.add('AUV8_trickshot_failed',
										send_update(mission=2, state=-1),
										transitions={'continue': 'Move_now'},
										autonomy={'continue': Autonomy.Off})

			# x:43 y:312
			OperatableStateMachine.add('Choose_sub',
										choose_your_character(submarine='AUV8'),
										transitions={'auv8': 'Move_to_gate_straight_AUV8', 'auv7': 'waiting_for_friend'},
										autonomy={'auv8': Autonomy.Off, 'auv7': Autonomy.Off})

			# x:25 y:237
			OperatableStateMachine.add('Coin_flip_completed',
										send_update(mission=0, state=2),
										transitions={'continue': 'Choose_sub'},
										autonomy={'continue': Autonomy.Off})

			# x:236 y:172
			OperatableStateMachine.add('Coin_flip_failed',
										send_update(mission=0, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:1053 y:327
			OperatableStateMachine.add('Get_update_of_AUV7',
										get_update(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'mission_array': 'mission_array'})

			# x:1086 y:110
			OperatableStateMachine.add('Move_now',
										synchro_send(),
										transitions={'continue': 'Move_to_gate_straight_3'},
										autonomy={'continue': Autonomy.Off})

			# x:1006 y:221
			OperatableStateMachine.add('Move_to_gate_straight_3',
										self.use_behavior(Move_to_gate_straightSM, 'Move_to_gate_straight_3'),
										transitions={'finished': 'Get_update_of_AUV7', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:246 y:385
			OperatableStateMachine.add('Move_to_gate_straight_AUV7',
										self.use_behavior(Move_to_gate_straightSM, 'Move_to_gate_straight_AUV7'),
										transitions={'finished': 'AUV7_gate_completed', 'failed': 'AUV7_gate_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:247 y:310
			OperatableStateMachine.add('Move_to_gate_straight_AUV8',
										self.use_behavior(Move_to_gate_straightSM, 'Move_to_gate_straight_AUV8'),
										transitions={'finished': 'AUV8_gate_completed', 'failed': 'AUV8_gate_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:455 y:668
			OperatableStateMachine.add('Path',
										self.use_behavior(vision_path_new_algoSM, 'Path'),
										transitions={'finished': 'AUV7_path_completed', 'failed': 'AUV7_path_failed', 'lost_target': 'AUV7_path_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'lost_target': Autonomy.Inherit})

			# x:31 y:160
			OperatableStateMachine.add('coin_flip_KY',
										self.use_behavior(coin_flip_KYSM, 'coin_flip_KY'),
										transitions={'finished': 'Coin_flip_completed', 'failed': 'Coin_flip_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1064 y:530
			OperatableStateMachine.add('get_update_AUV8',
										get_update(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off},
										remapping={'mission_array': 'mission_array'})

			# x:0 y:591
			OperatableStateMachine.add('taking_over_trickshot',
										takeover_mission(mission_id=2),
										transitions={'takeover': 'trickshot_if_not_done', 'already_done': 'Path'},
										autonomy={'takeover': Autonomy.Off, 'already_done': Autonomy.Off})

			# x:664 y:119
			OperatableStateMachine.add('trickshot',
										self.use_behavior(trickshotSM, 'trickshot'),
										transitions={'finished': 'AUV8_trickshot_completed', 'failed': 'AUV8_trickshot_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:245 y:497
			OperatableStateMachine.add('trickshot_if_not_done',
										self.use_behavior(trickshotSM, 'trickshot_if_not_done'),
										transitions={'finished': 'AUV7_trickshot_completed', 'failed': 'AUV7_trickshot_failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:44 y:417
			OperatableStateMachine.add('waiting_for_friend',
										synchro_receive(timeout=120),
										transitions={'continue': 'Move_to_gate_straight_AUV7', 'timeout': 'Move_to_gate_straight_AUV7'},
										autonomy={'continue': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:0 y:493
			OperatableStateMachine.add('AUV7_gate_completed',
										send_update(mission=1, state=2),
										transitions={'continue': 'taking_over_trickshot'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
