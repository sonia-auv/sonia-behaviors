#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_hardware_states.wait_mission import wait_mission
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.set_control_mode import set_control_mode
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 22 2022
@author: LAMARRE
'''
class trick_shotSM(Behavior):
	'''
	trickshot 6DOF
	'''


	def __init__(self):
		super(trick_shotSM, self).__init__()
		self.name = 'trick_shot'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:242 y:191
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:72 y:56
			OperatableStateMachine.add('start',
										wait_mission(),
										transitions={'continue': 'mode', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:333 y:52
			OperatableStateMachine.add('mode',
										set_control_mode(mode=10, timeout=5),
										transitions={'continue': 'init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:524 y:137
			OperatableStateMachine.add('pose1',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0.6, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=1, precision=0, long_rotation=False),
										transitions={'continue': 'pose2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:524 y:221
			OperatableStateMachine.add('pose2',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=90, orientationZ=0, frame=1, speed=2, precision=0, long_rotation=False),
										transitions={'continue': 'pose3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:523 y:306
			OperatableStateMachine.add('pose3',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=180, orientationY=0, orientationZ=0, frame=1, speed=2, precision=0, long_rotation=False),
										transitions={'continue': 'pose4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:524 y:390
			OperatableStateMachine.add('pose4',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=180, orientationY=0, orientationZ=0, frame=1, speed=2, precision=0, long_rotation=False),
										transitions={'continue': 'pose5'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:523 y:483
			OperatableStateMachine.add('pose5',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=180, frame=1, speed=2, precision=0, long_rotation=False),
										transitions={'continue': 'pose6'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:521 y:578
			OperatableStateMachine.add('pose6',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=180, frame=1, speed=2, precision=0, long_rotation=False),
										transitions={'continue': 'pose7'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:520 y:661
			OperatableStateMachine.add('pose7',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=-90, orientationZ=0, frame=1, speed=2, precision=0, long_rotation=False),
										transitions={'continue': 'send'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:286 y:359
			OperatableStateMachine.add('send',
										send_to_planner(),
										transitions={'continue': 'tg', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:126 y:360
			OperatableStateMachine.add('tg',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'finished', 'target_not_reached': 'finished', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:504 y:53
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'pose1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
