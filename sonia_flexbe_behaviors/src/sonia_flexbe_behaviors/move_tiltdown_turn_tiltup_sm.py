#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 30 2022
@author: GS
'''
class move_tiltdown_turn_tiltupSM(Behavior):
	'''
	move_tiltdown_turn_tiltup
	'''


	def __init__(self):
		super(move_tiltdown_turn_tiltupSM, self).__init__()
		self.name = 'move_tiltdown_turn_tiltup'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:626 y:209, x:382 y:211
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'down'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:514 y:420
			OperatableStateMachine.add('is_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'wait', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:62 y:198
			OperatableStateMachine.add('planner',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:453 y:36
			OperatableStateMachine.add('turn',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=90, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'up'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:669 y:42
			OperatableStateMachine.add('up',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=20, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:325 y:316
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'is_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:227 y:53
			OperatableStateMachine.add('down',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=-20, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'turn'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
