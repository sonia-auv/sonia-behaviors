#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_com_states.send_update import send_update
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 17 2022
@author: FA
'''
class CoinFlipGateSM(Behavior):
	'''
	Orient to gate for coin flip task and starts to go to the gate.
	'''


	def __init__(self):
		super(CoinFlipGateSM, self).__init__()
		self.name = 'CoinFlip-Gate'

		# parameters of this behavior
		self.add_parameter('orientation_to_gate', 0)
		self.add_parameter('dive_depth', 1)
		self.add_parameter('distance_to_gate', 4)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:173 y:326, x:799 y:640
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:138 y:52
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'dive'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_traj'})

			# x:404 y:52
			OperatableStateMachine.add('dive',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=self.dive_depth, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'turn_to_gate'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'input_traj', 'trajectory': 'trajectory'})

			# x:747 y:341
			OperatableStateMachine.add('failed_coin_flip',
										send_update(mission=0, state=-1),
										transitions={'continue': 'failed_gate'},
										autonomy={'continue': Autonomy.Off})

			# x:751 y:491
			OperatableStateMachine.add('failed_gate',
										send_update(mission=1, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:1053 y:262
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'failed_coin_flip'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:1016 y:49
			OperatableStateMachine.add('move_to_gate',
										manual_add_pose_to_trajectory(positionX=self.distance_to_gate, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'input_traj', 'trajectory': 'trajectory'})

			# x:381 y:153
			OperatableStateMachine.add('success_coin_flip',
										send_update(mission=0, state=2),
										transitions={'continue': 'success_gate'},
										autonomy={'continue': Autonomy.Off})

			# x:142 y:152
			OperatableStateMachine.add('success_gate',
										send_update(mission=1, state=2),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:705 y:52
			OperatableStateMachine.add('turn_to_gate',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=self.orientation_to_gate, frame=2, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'move_to_gate'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:729 y:194
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'success_coin_flip', 'target_not_reached': 'check_moving', 'error': 'failed_coin_flip'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:375 y:337
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'success_coin_flip', 'moving': 'wait', 'error': 'failed_coin_flip'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
