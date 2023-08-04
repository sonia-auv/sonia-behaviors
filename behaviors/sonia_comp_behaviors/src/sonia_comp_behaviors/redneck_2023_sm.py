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
from sonia_hardware_states.wait_mission import wait_mission
from sonia_navigation_states.set_control_mode import set_control_mode
from sonia_vision_states.get_buoy import get_buoy
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
		self.add_behavior(SinglePoseMoveSM, 'Dive to 1.3m move 3m')
		self.add_behavior(SinglePoseMoveSM, 'Move 5x, -0.2z after Coin Flip')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move_2')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move_2_2')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move_3')
		self.add_behavior(coin_flip_tareSM, 'coin_flip_tare')
		self.add_behavior(trick_shot_pitch_rollSM, 'trick_shot_pitch_roll')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 607 34 
		# Move forward to get to the gate. Rise by 0.2 to see the target pannel on the gate.



	def create(self):
		# x:33 y:467, x:408 y:373
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.Real_Run = False

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:89
			OperatableStateMachine.add('wait mission',
										wait_mission(),
										transitions={'continue': 'coin_flip_tare'},
										autonomy={'continue': Autonomy.Off})

			# x:657 y:262
			OperatableStateMachine.add('Dive to 1.3m move 3m',
										self.use_behavior(SinglePoseMoveSM, 'Dive to 1.3m move 3m',
											parameters={'positionX': 6, 'positionY': 0.0, 'positionZ': 0.5, 'orientationZ': 0.0}),
										transitions={'finished': 'trick_shot_pitch_roll', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:380 y:478
			OperatableStateMachine.add('Get Buoy',
										get_buoy(),
										transitions={'success': 'Single Pose Move', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'average_size': 'average_size', 'average_ctr': 'average_ctr'})

			# x:374 y:28
			OperatableStateMachine.add('Move 5x, -0.2z after Coin Flip',
										self.use_behavior(SinglePoseMoveSM, 'Move 5x, -0.2z after Coin Flip',
											parameters={'positionX': 4, 'positionY': 0.0, 'positionZ': -0.4, 'orientationZ': 0.0}),
										transitions={'finished': 'Align Gate Angle', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:182 y:436
			OperatableStateMachine.add('Single Pose Move',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move',
											parameters={'positionX': 1.5}),
										transitions={'finished': 'Single Pose Move_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:237 y:557
			OperatableStateMachine.add('Single Pose Move_2',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move_2',
											parameters={'positionX': -1.5}),
										transitions={'finished': 'Single Pose Move_3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:82 y:295
			OperatableStateMachine.add('Single Pose Move_2_2',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move_2_2',
											parameters={'positionX': -1.5}),
										transitions={'finished': 'kill', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:20 y:565
			OperatableStateMachine.add('Single Pose Move_3',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move_3',
											parameters={'positionX': 1.5, 'positionZ': 0.5}),
										transitions={'finished': 'Single Pose Move_2_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:134 y:21
			OperatableStateMachine.add('coin_flip_tare',
										self.use_behavior(coin_flip_tareSM, 'coin_flip_tare',
											parameters={'orientation_to_gate': 0, 'dive_depth': 0, 'activate_coin_flip': True}),
										transitions={'finished': 'Move 5x, -0.2z after Coin Flip', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:30 y:151
			OperatableStateMachine.add('kill',
										set_control_mode(mode=0, timeout=5),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:644 y:404
			OperatableStateMachine.add('trick_shot_pitch_roll',
										self.use_behavior(trick_shot_pitch_rollSM, 'trick_shot_pitch_roll'),
										transitions={'finished': 'Get Buoy', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:651 y:118
			OperatableStateMachine.add('Align Gate Angle',
										self.use_behavior(AlignGateAngleSM, 'Align Gate Angle'),
										transitions={'finished': 'Dive to 1.3m move 3m', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
