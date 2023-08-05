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
from sonia_flexbe_behaviors.bullshit_sm import bullshitSM
from sonia_hardware_states.wait_mission import wait_mission
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Aug 05 2023
@author: Nimai
'''
class BullshitMagicSM(Behavior):
	'''
	hehe
	'''


	def __init__(self):
		super(BullshitMagicSM, self).__init__()
		self.name = 'Bullshit Magic'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(SinglePoseMoveSM, 'Single Pose Move')
		self.add_behavior(bullshitSM, 'bullshit Gate')
		self.add_behavior(coin_flip_tareSM, 'coin_flip_tare')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:545 y:496, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Mission',
										wait_mission(),
										transitions={'continue': 'coin_flip_tare'},
										autonomy={'continue': Autonomy.Off})

			# x:492 y:233
			OperatableStateMachine.add('Single Pose Move',
										self.use_behavior(SinglePoseMoveSM, 'Single Pose Move',
											parameters={'positionX': 3, 'positionY': 0.0, 'positionZ': 0.0, 'orientationZ': 0.0}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:492 y:82
			OperatableStateMachine.add('bullshit Gate',
										self.use_behavior(bullshitSM, 'bullshit Gate',
											parameters={'target': "Gate"}),
										transitions={'finished': 'Single Pose Move', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:255 y:42
			OperatableStateMachine.add('coin_flip_tare',
										self.use_behavior(coin_flip_tareSM, 'coin_flip_tare',
											parameters={'orientation_to_gate': 0, 'dive_depth': 1, 'activate_coin_flip': True}),
										transitions={'finished': 'bullshit Gate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
