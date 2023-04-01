#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_flexbe_behaviors.move_tiltdown_turn_tiltup_sm import move_tiltdown_turn_tiltupSM
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Feb 02 2022
@author: GS
'''
class full_turn_tiltedSM(Behavior):
	'''
	Behavior to move the submarine with 360 4 stops
	'''


	def __init__(self):
		super(full_turn_tiltedSM, self).__init__()
		self.name = 'full_turn_tilted'

		# parameters of this behavior
		self.add_parameter('wait_time', 2)

		# references to used behaviors
		self.add_behavior(moveSM, 'move')
		self.add_behavior(move_tiltdown_turn_tiltupSM, 'move_tiltdown_turn_tiltup')
		self.add_behavior(move_tiltdown_turn_tiltupSM, 'move_tiltdown_turn_tiltup_2')
		self.add_behavior(move_tiltdown_turn_tiltupSM, 'move_tiltdown_turn_tiltup_3')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:894 y:502, x:869 y:326
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:65 y:32
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:45 y:138
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': 0, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 20, 'orientationZ': 0, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'wait', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:438 y:123
			OperatableStateMachine.add('move_tiltdown_turn_tiltup',
										self.use_behavior(move_tiltdown_turn_tiltupSM, 'move_tiltdown_turn_tiltup'),
										transitions={'finished': 'wait2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:423 y:207
			OperatableStateMachine.add('move_tiltdown_turn_tiltup_2',
										self.use_behavior(move_tiltdown_turn_tiltupSM, 'move_tiltdown_turn_tiltup_2'),
										transitions={'finished': 'wait3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:373 y:314
			OperatableStateMachine.add('move_tiltdown_turn_tiltup_3',
										self.use_behavior(move_tiltdown_turn_tiltupSM, 'move_tiltdown_turn_tiltup_3'),
										transitions={'finished': 'wait4', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:611 y:481
			OperatableStateMachine.add('send',
										send_to_planner(),
										transitions={'continue': 'wait_target', 'failed': 'tilt4'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:373 y:430
			OperatableStateMachine.add('tilt4',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=-20, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:295 y:136
			OperatableStateMachine.add('wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'move_tiltdown_turn_tiltup'},
										autonomy={'done': Autonomy.Off})

			# x:92 y:227
			OperatableStateMachine.add('wait2',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'move_tiltdown_turn_tiltup_2'},
										autonomy={'done': Autonomy.Off})

			# x:77 y:337
			OperatableStateMachine.add('wait3',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'move_tiltdown_turn_tiltup_3'},
										autonomy={'done': Autonomy.Off})

			# x:82 y:439
			OperatableStateMachine.add('wait4',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'tilt4'},
										autonomy={'done': Autonomy.Off})

			# x:692 y:574
			OperatableStateMachine.add('wait_target',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'are_you_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:676 y:376
			OperatableStateMachine.add('are_you_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'wait_target', 'error': 'send'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
