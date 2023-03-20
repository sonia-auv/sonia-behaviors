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
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Mar 07 2023
@author: Nimai Jariwala
'''
class Mission_2_NimaiSM(Behavior):
	'''
	Exercise Mission 2
	'''


	def __init__(self):
		super(Mission_2_NimaiSM, self).__init__()
		self.name = 'Mission_2_Nimai'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:624 y:506, x:988 y:146
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:285 y:358
		_sm_adding_poses_0 = OperatableStateMachine(outcomes=['continue'], input_keys=['trajectory'], output_keys=['trajectory'])

		with _sm_adding_poses_0:
			# x:30 y:40
			OperatableStateMachine.add('Dive 1 m',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=1.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'turn 90 deg right'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:614 y:39
			OperatableStateMachine.add('Forward 2m',
										manual_add_pose_to_trajectory(positionX=2.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'move right 3m'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:626 y:302
			OperatableStateMachine.add('Forward 2m (backwards)',
										manual_add_pose_to_trajectory(positionX=2.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'continue'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:886 y:294
			OperatableStateMachine.add('Move right 3m (backwards)',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=3.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Forward 2m (backwards)'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:893 y:45
			OperatableStateMachine.add('move right 3m',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=3.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': '180 deg turn'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:335 y:38
			OperatableStateMachine.add('turn 90 deg right',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=90.0, frame=2, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Forward 2m'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:1110 y:165
			OperatableStateMachine.add('180 deg turn',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=180.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Move right 3m (backwards)'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})



		with _state_machine:
			# x:145 y:36
			OperatableStateMachine.add('Start',
										activate_behavior(activate=True),
										transitions={'activate': 'Init Trajectory List', 'desactivate': 'finished'},
										autonomy={'activate': Autonomy.Off, 'desactivate': Autonomy.Off})

			# x:376 y:51
			OperatableStateMachine.add('Init Trajectory List',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'Adding Poses'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:624 y:245
			OperatableStateMachine.add('Send Path to planner',
										send_to_planner(),
										transitions={'continue': 'Wait for path to finish', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:832 y:318
			OperatableStateMachine.add('Wait for path to finish',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:643 y:40
			OperatableStateMachine.add('Adding Poses',
										_sm_adding_poses_0,
										transitions={'continue': 'Send Path to planner'},
										autonomy={'continue': Autonomy.Inherit},
										remapping={'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
