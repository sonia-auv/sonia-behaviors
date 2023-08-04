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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Aug 03 2023
@author: Nimai
'''
class Redneck2023SM(Behavior):
	'''
	oh god I hope this works
	'''


	def __init__(self):
		super(Redneck2023SM, self).__init__()
		self.name = 'Redneck 2023'

		# parameters of this behavior
		self.add_parameter('real_run', False)

		# references to used behaviors
		self.add_behavior(AlignGateAngleSM, 'Align Gate Angle')
		self.add_behavior(SinglePoseMoveSM, 'Dive to 1.5m')
		self.add_behavior(SinglePoseMoveSM, 'Move 3x, -0.2z after Coin Flip')
		self.add_behavior(coin_flip_tareSM, 'coin_flip_tare')
		self.add_behavior(trick_shot_pitch_rollSM, 'trick_shot_pitch_roll')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 579 86 
		# Move forward to get to the gate. Rise by 0.2 to see the target pannel on the gate.



	def create(self):
		# x:33 y:467, x:408 y:373
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.Real_Run = False

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:375 y:118
			OperatableStateMachine.add('coin_flip_tare',
										self.use_behavior(coin_flip_tareSM, 'coin_flip_tare',
											parameters={'orientation_to_gate': 0, 'dive_depth': 1, 'activate_coin_flip': False}),
										transitions={'finished': 'Move 3x, -0.2z after Coin Flip', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:705 y:411
			OperatableStateMachine.add('Dive to 1.5m',
										self.use_behavior(SinglePoseMoveSM, 'Dive to 1.5m',
											parameters={'positionX': 0.0, 'positionY': 0.0, 'positionZ': 0.7, 'orientationZ': 0.0}),
										transitions={'finished': 'trick_shot_pitch_roll', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:582 y:130
			OperatableStateMachine.add('Move 3x, -0.2z after Coin Flip',
										self.use_behavior(SinglePoseMoveSM, 'Move 3x, -0.2z after Coin Flip',
											parameters={'positionX': 3, 'positionY': 0.0, 'positionZ': -0.2, 'orientationZ': 0.0}),
										transitions={'finished': 'Align Gate Angle', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:547 y:487
			OperatableStateMachine.add('trick_shot_pitch_roll',
										self.use_behavior(trick_shot_pitch_rollSM, 'trick_shot_pitch_roll'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:695 y:255
			OperatableStateMachine.add('Align Gate Angle',
										self.use_behavior(AlignGateAngleSM, 'Align Gate Angle'),
										transitions={'finished': 'Dive to 1.5m', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
