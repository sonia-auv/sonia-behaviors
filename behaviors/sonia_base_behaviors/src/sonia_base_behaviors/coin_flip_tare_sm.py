#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.activate_behavior import activate_behavior
from sonia_hardware_states.wait_mission import wait_mission
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on 28 May 2023
@author: William Brouillard, Nimai Jariwala
'''
class coin_flip_tareSM(Behavior):
	'''
	Orient to gate for coin flip task.
	'''


	def __init__(self):
		super(coin_flip_tareSM, self).__init__()
		self.name = 'coin_flip_tare'

		# parameters of this behavior
		self.add_parameter('orientation_to_gate', 0)
		self.add_parameter('dive_depth', 1)
		self.add_parameter('activate_coin_flip', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:765 y:559, x:215 y:565
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:38 y:117
			OperatableStateMachine.add('Mission',
										wait_mission(),
										transitions={'continue': 'activate_coin_flip'},
										autonomy={'continue': Autonomy.Off})

			# x:236 y:38
			OperatableStateMachine.add('activate_coin_flip',
										activate_behavior(activate=self.activate_coin_flip),
										transitions={'activate': 'init_traj', 'desactivate': 'finished'},
										autonomy={'activate': Autonomy.Off, 'desactivate': Autonomy.Off})

			# x:454 y:622
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'wait', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:444 y:152
			OperatableStateMachine.add('dive',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0.3, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'turn'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'input_traj', 'trajectory': 'trajectory'})

			# x:461 y:47
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'dive'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_traj'})

			# x:444 y:381
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:433 y:259
			OperatableStateMachine.add('turn',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=self.orientation_to_gate, frame=2, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:444 y:495
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'finished', 'target_not_reached': 'check_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
