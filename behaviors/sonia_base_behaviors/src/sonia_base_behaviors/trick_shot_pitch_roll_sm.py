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
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun May 28 2023
@author: Nimai Jariwala
'''
class trick_shot_pitch_rollSM(Behavior):
	'''
	Do the candle, spin, finish the backflip.
	'''


	def __init__(self):
		super(trick_shot_pitch_rollSM, self).__init__()
		self.name = 'trick_shot_pitch_roll'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:139 y:768, x:176 y:656
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:48 y:44
			OperatableStateMachine.add('Init Traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'Pitch 90 y'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:42 y:387
			OperatableStateMachine.add('Pitch 180 ',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=180, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Send Moves'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'pitch2', 'trajectory': 'pitch3'})

			# x:356 y:339
			OperatableStateMachine.add('Pitch 90 ',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=90, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Pitch 180 '},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'roll2', 'trajectory': 'pitch2'})

			# x:62 y:126
			OperatableStateMachine.add('Pitch 90 y',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=90, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Roll 180 x'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'pitch1'})

			# x:357 y:192
			OperatableStateMachine.add('Roll 180 x',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=180, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Roll again 180 x'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'pitch1', 'trajectory': 'roll1'})

			# x:58 y:253
			OperatableStateMachine.add('Roll again 180 x',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=180, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Pitch 90 '},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'roll1', 'trajectory': 'roll2'})

			# x:90 y:497
			OperatableStateMachine.add('Send Moves',
										send_to_planner(),
										transitions={'continue': 'Wait for finish', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'pitch3'})

			# x:406 y:554
			OperatableStateMachine.add('Wait for finish',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
