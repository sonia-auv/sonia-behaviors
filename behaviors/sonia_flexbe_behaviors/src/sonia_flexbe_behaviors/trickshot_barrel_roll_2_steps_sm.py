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
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun May 14 2023
@author: Nimai Jariwala
'''
class trickshotbarrelroll2stepsSM(Behavior):
	'''
	Do a barrel roll by moving, stoping, rolling, moving.
	'''


	def __init__(self):
		super(trickshotbarrelroll2stepsSM, self).__init__()
		self.name = 'trickshot barrel roll 2 steps'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:425, x:130 y:425
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:100 y:88
			OperatableStateMachine.add('Start traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'Move Forwards (step 1)'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:358 y:428
			OperatableStateMachine.add('Move Forwards (step 2)',
										manual_add_pose_to_trajectory(positionX=2, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send positions'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'roll2', 'trajectory': 'move2'})

			# x:365 y:242
			OperatableStateMachine.add('Roll (1)',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=180, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Roll (2)'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'move1', 'trajectory': 'roll1'})

			# x:382 y:328
			OperatableStateMachine.add('Roll (2)',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=180, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Move Forwards (step 2)'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'roll1', 'trajectory': 'roll2'})

			# x:143 y:289
			OperatableStateMachine.add('send positions',
										send_to_planner(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'move2'})

			# x:356 y:124
			OperatableStateMachine.add('Move Forwards (step 1)',
										manual_add_pose_to_trajectory(positionX=2, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Roll (1)'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'move1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
