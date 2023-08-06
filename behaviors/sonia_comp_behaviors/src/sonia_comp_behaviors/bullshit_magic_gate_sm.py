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
from sonia_hardware_states.wait_mission import wait_mission
from sonia_vision_states.check_side_gate import check_gate_side
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Aug 05 2023
@author: Nimai
'''
class BullshitMagicgateSM(Behavior):
	'''
	hehe
	'''


	def __init__(self):
		super(BullshitMagicgateSM, self).__init__()
		self.name = 'Bullshit Magic gate'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move_2')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move_2_2')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move_3')
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move_4')
		self.add_behavior(coin_flip_tareSM, 'coin_flip_tare')
		self.add_behavior(trick_shot_pitch_rollSM, 'trick_shot_pitch_roll')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:545 y:496, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['gate_side'])
		_state_machine.userdata.gate_side = "Earth"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:163 y:133
			OperatableStateMachine.add('Mission',
										wait_mission(),
										transitions={'continue': 'coin_flip_tare'},
										autonomy={'continue': Autonomy.Off})

			# x:298 y:427
			OperatableStateMachine.add('Single Pose Move',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move',
											parameters={'positionX': 0.0, 'positionY': 0.0, 'positionZ': 0.0, 'orientationZ': 10}),
										transitions={'finished': 'Single Pose Move_3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:450 y:38
			OperatableStateMachine.add('Single Pose Move_2',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move_2',
											parameters={'positionX': 3}),
										transitions={'finished': 'gate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:667 y:208
			OperatableStateMachine.add('Single Pose Move_2_2',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move_2_2',
											parameters={'positionX': 7}),
										transitions={'finished': 'trick_shot_pitch_roll', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:298 y:495
			OperatableStateMachine.add('Single Pose Move_3',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move_3',
											parameters={'positionX': 0.0, 'positionY': 0.0, 'positionZ': 0.0, 'orientationZ': 20}),
										transitions={'finished': 'Single Pose Move_4', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:298 y:593
			OperatableStateMachine.add('Single Pose Move_4',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move_4',
											parameters={'positionX': 0.0, 'positionY': 0.0, 'positionZ': 1.5, 'orientationZ': -10}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:143 y:49
			OperatableStateMachine.add('coin_flip_tare',
										self.use_behavior(coin_flip_tareSM, 'coin_flip_tare',
											parameters={'orientation_to_gate': 0, 'dive_depth': 1, 'activate_coin_flip': True}),
										transitions={'finished': 'Single Pose Move_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:702 y:62
			OperatableStateMachine.add('gate',
										check_gate_side(nb_img=10),
										transitions={'success': 'Single Pose Move_2_2', 'failed': 'Single Pose Move_2_2'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'gate_side': 'gate_side'})

			# x:475 y:332
			OperatableStateMachine.add('trick_shot_pitch_roll',
										self.use_behavior(trick_shot_pitch_rollSM, 'trick_shot_pitch_roll'),
										transitions={'finished': 'Single Pose Move', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
