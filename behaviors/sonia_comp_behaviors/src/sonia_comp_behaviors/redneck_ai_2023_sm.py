#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_base_behaviors.coin_flip_tare_sm import coin_flip_tareSM
from sonia_base_behaviors.single_pose_move_sm import SinglePoseMoveSM
from sonia_base_behaviors.trick_shot_pitch_roll_sm import trick_shot_pitch_rollSM
from sonia_complex_behaviors.align_gate_angle_sm import AlignGateAngleSM
from sonia_vision_states.calc_ctr_ai import calculate_ctr_ai
from sonia_vision_states.get_ai import get_ai
from sonia_vision_states.get_buoy import get_buoy
from sonia_vision_states.move_to_ctr import move_to_ctr
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 04 2023
@author: Nimai
'''
class RedneckAI2023SM(Behavior):
	'''
	Redneck stuff but with AI
	'''


	def __init__(self):
		super(RedneckAI2023SM, self).__init__()
		self.name = 'Redneck AI 2023'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(AlignGateAngleSM, 'Align Gate Angle')
		self.add_behavior(SinglePoseMoveSM, 'Dive to 1.3m move 3m')
		self.add_behavior(SinglePoseMoveSM, 'Move 5x, -0.2z after Coin Flip')
		self.add_behavior(coin_flip_tareSM, 'coin_flip_tare')
		self.add_behavior(trick_shot_pitch_rollSM, 'trick_shot_pitch_roll')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:294 y:582, x:130 y:458
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:134 y:21
			OperatableStateMachine.add('coin_flip_tare',
										self.use_behavior(coin_flip_tareSM, 'coin_flip_tare',
											parameters={'orientation_to_gate': 0, 'dive_depth': 1, 'activate_coin_flip': True}),
										transitions={'finished': 'Get gate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:657 y:262
			OperatableStateMachine.add('Dive to 1.3m move 3m',
										self.use_behavior(SinglePoseMoveSM, 'Dive to 1.3m move 3m',
											parameters={'positionX': 6, 'positionY': 0.0, 'positionZ': 0.5, 'orientationZ': 0.0}),
										transitions={'finished': 'trick_shot_pitch_roll', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:380 y:478
			OperatableStateMachine.add('Get Buoy',
										get_buoy(),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'average_size': 'average_size', 'average_ctr': 'average_ctr'})

			# x:329 y:123
			OperatableStateMachine.add('Get gate',
										get_ai(class_name="Gate", nb_img=10),
										transitions={'success': 'calc avg', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ai_pos': 'ai_pos'})

			# x:496 y:342
			OperatableStateMachine.add('Move',
										move_to_ctr(),
										transitions={'success': 'Move 5x, -0.2z after Coin Flip'},
										autonomy={'success': Autonomy.Off},
										remapping={'obj_ctr': 'obj_ctr'})

			# x:549 y:15
			OperatableStateMachine.add('Move 5x, -0.2z after Coin Flip',
										self.use_behavior(SinglePoseMoveSM, 'Move 5x, -0.2z after Coin Flip',
											parameters={'positionX': 3, 'positionY': 0.0, 'positionZ': -0.4, 'orientationZ': 0.0}),
										transitions={'finished': 'Align Gate Angle', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:376 y:249
			OperatableStateMachine.add('calc avg',
										calculate_ctr_ai(),
										transitions={'success': 'Move'},
										autonomy={'success': Autonomy.Off},
										remapping={'ai_pos': 'ai_pos', 'obj_ctr': 'obj_ctr'})

			# x:644 y:404
			OperatableStateMachine.add('trick_shot_pitch_roll',
										self.use_behavior(trick_shot_pitch_rollSM, 'trick_shot_pitch_roll'),
										transitions={'finished': 'Get Buoy', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:792 y:101
			OperatableStateMachine.add('Align Gate Angle',
										self.use_behavior(AlignGateAngleSM, 'Align Gate Angle'),
										transitions={'finished': 'Dive to 1.3m move 3m', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
