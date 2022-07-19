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
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.set_control_mode import set_control_mode
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_navigation_states.yaw_orbit_from_given_point import yaw_orbit_from_given_point
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed May 18 2022
@author: Willy Kao
'''
class mission_bags_binsSM(Behavior):
	'''
	Mission to create bags for the bins
	'''


	def __init__(self):
		super(mission_bags_binsSM, self).__init__()
		self.name = 'mission_bags_bins'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1275 y:612, x:97 y:678
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:49 y:100
			OperatableStateMachine.add('wait_mission',
										wait_mission(),
										transitions={'continue': 'set_ctrl_mode', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:396 y:231
			OperatableStateMachine.add('go_down_1',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0.5, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'rotate_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'rotate_0', 'trajectory': 'go_down_1'})

			# x:404 y:429
			OperatableStateMachine.add('go_down_2',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0.5, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'rotate_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'rotate_1', 'trajectory': 'go_down_2'})

			# x:642 y:38
			OperatableStateMachine.add('go_down_3',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0.5, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'rotate_3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'rotate_2', 'trajectory': 'go_down_3'})

			# x:661 y:244
			OperatableStateMachine.add('go_down_4',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0.5, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'rotate_4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'rotate_3', 'trajectory': 'go_down_4'})

			# x:668 y:431
			OperatableStateMachine.add('go_down_5',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0.5, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'rotate_5'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'rotate_4', 'trajectory': 'go_down_5'})

			# x:181 y:322
			OperatableStateMachine.add('init_traj',
										init_trajectory(InterpolationMethod=0),
										transitions={'continue': 'go_down_0'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'init_traj'})

			# x:386 y:133
			OperatableStateMachine.add('rotate_0',
										yaw_orbit_from_given_point(pointX=0.2415, pointY=0, rotation=360, speed=1),
										transitions={'continue': 'go_down_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'go_down_0', 'trajectory': 'rotate_0'})

			# x:396 y:331
			OperatableStateMachine.add('rotate_1',
										yaw_orbit_from_given_point(pointX=0.2415, pointY=0, rotation=360, speed=1),
										transitions={'continue': 'go_down_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'go_down_1', 'trajectory': 'rotate_1'})

			# x:405 y:526
			OperatableStateMachine.add('rotate_2',
										yaw_orbit_from_given_point(pointX=0.2415, pointY=0, rotation=360, speed=1),
										transitions={'continue': 'go_down_3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'go_down_2', 'trajectory': 'rotate_2'})

			# x:639 y:151
			OperatableStateMachine.add('rotate_3',
										yaw_orbit_from_given_point(pointX=0.2415, pointY=0, rotation=360, speed=1),
										transitions={'continue': 'go_down_4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'go_down_3', 'trajectory': 'rotate_3'})

			# x:648 y:338
			OperatableStateMachine.add('rotate_4',
										yaw_orbit_from_given_point(pointX=0.2415, pointY=0, rotation=360, speed=1),
										transitions={'continue': 'go_down_5'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'go_down_4', 'trajectory': 'rotate_4'})

			# x:651 y:522
			OperatableStateMachine.add('rotate_5',
										yaw_orbit_from_given_point(pointX=0.2415, pointY=0, rotation=360, speed=1),
										transitions={'continue': 'send_to_planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'go_down_5', 'trajectory': 'rotate_5'})

			# x:1046 y:558
			OperatableStateMachine.add('send_to_planner',
										send_to_planner(),
										transitions={'continue': 'wait_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'rotate_5'})

			# x:127 y:213
			OperatableStateMachine.add('set_ctrl_mode',
										set_control_mode(mode=10, timeout=2),
										transitions={'continue': 'init_traj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1043 y:651
			OperatableStateMachine.add('wait_target',
										wait_target_reached(),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:383 y:36
			OperatableStateMachine.add('go_down_0',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0.5, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'rotate_0'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'init_traj', 'trajectory': 'go_down_0'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
