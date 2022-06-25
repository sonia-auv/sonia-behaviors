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
Created on Fri Jun 24 2022
@author: KYGS
'''
class trickshotSM(Behavior):
	'''
	Simple_trickshot
	'''


	def __init__(self):
		super(trickshotSM, self).__init__()
		self.name = 'trickshot'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:578 y:531
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:88 y:82
			OperatableStateMachine.add('init_trajectory',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'traj1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'new_traj'})

			# x:257 y:649
			OperatableStateMachine.add('is_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'traget_reached', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:278 y:439
			OperatableStateMachine.add('send_to_planner',
										send_to_planner(),
										transitions={'continue': 'traget_reached', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'traj'})

			# x:271 y:548
			OperatableStateMachine.add('traget_reached',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'finished', 'target_not_reached': 'is_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:270 y:82
			OperatableStateMachine.add('traj1',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=90, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'traj2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'new_traj', 'trajectory': 'traj'})

			# x:274 y:180
			OperatableStateMachine.add('traj2',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=180, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'traj3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj', 'trajectory': 'traj'})

			# x:270 y:263
			OperatableStateMachine.add('traj3',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=180, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'traj4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj', 'trajectory': 'traj'})

			# x:272 y:355
			OperatableStateMachine.add('traj4',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=-90, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send_to_planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj', 'trajectory': 'traj'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
