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
Created on Tue Feb 15 2022
@author: FA
'''
class PrequalifSM(Behavior):
	'''
	Prequalification for Robosub 2022 and Robosub 2023
	'''


	def __init__(self):
		super(PrequalifSM, self).__init__()
		self.name = 'Prequalif'

		# parameters of this behavior
		self.add_parameter('angle_gate', 0)
		self.add_parameter('distance_start_loop', 7.572)
		self.add_parameter('interpolation_method', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:732 y:653, x:1261 y:531, x:938 y:344
		_state_machine = OperatableStateMachine(outcomes=['finished', 'error', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:11 y:71
			OperatableStateMachine.add('mission_switch',
										wait_mission(),
										transitions={'continue': 'set mode'},
										autonomy={'continue': Autonomy.Off})

			# x:553 y:54
			OperatableStateMachine.add('pose 1',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0.76, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'pose 2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'empty_traj', 'trajectory': 'traj1'})

			# x:79 y:303
			OperatableStateMachine.add('pose 10',
										manual_add_pose_to_trajectory(positionX=3.849, positionY=-1.364, positionZ=0.304, orientationX=0, orientationY=0, orientationZ=-42, frame=1, speed=1, precision=0, long_rotation=False),
										transitions={'continue': 'pose 12'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj9', 'trajectory': 'traj10'})

			# x:284 y:306
			OperatableStateMachine.add('pose 12',
										manual_add_pose_to_trajectory(positionX=self.distance_start_loop, positionY=-1, positionZ=0.2, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=1, precision=0, long_rotation=False),
										transitions={'continue': 'send to planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj10', 'trajectory': 'traj12'})

			# x:733 y:65
			OperatableStateMachine.add('pose 2',
										manual_add_pose_to_trajectory(positionX=0.5, positionY=0, positionZ=.5, orientationX=0, orientationY=0, orientationZ=self.angle_gate, frame=2, speed=1, precision=0, long_rotation=False),
										transitions={'continue': 'pose4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj1', 'trajectory': 'traj2'})

			# x:901 y:200
			OperatableStateMachine.add('pose 5',
										manual_add_pose_to_trajectory(positionX=3.849, positionY=-1.364, positionZ=-0.304, orientationX=0, orientationY=0, orientationZ=-36, frame=1, speed=1, precision=0, long_rotation=False),
										transitions={'continue': 'pose 6'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj4', 'trajectory': 'traj5'})

			# x:716 y:202
			OperatableStateMachine.add('pose 6',
										manual_add_pose_to_trajectory(positionX=1.079, positionY=0.383, positionZ=-0.196, orientationX=0, orientationY=0, orientationZ=39.23, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'pose 7'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj5', 'trajectory': 'traj6'})

			# x:503 y:201
			OperatableStateMachine.add('pose 7',
										manual_add_pose_to_trajectory(positionX=1.75, positionY=1.721, positionZ=-0.314, orientationX=0, orientationY=0, orientationZ=90, frame=1, speed=1, precision=0, long_rotation=False),
										transitions={'continue': 'pose 8'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj6', 'trajectory': 'traj7'})

			# x:281 y:200
			OperatableStateMachine.add('pose 8',
										manual_add_pose_to_trajectory(positionX=1.721, positionY=1.75, positionZ=0.314, orientationX=0, orientationY=0, orientationZ=90, frame=1, speed=1, precision=0, long_rotation=False),
										transitions={'continue': 'pose 9'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj7', 'trajectory': 'traj8'})

			# x:19 y:178
			OperatableStateMachine.add('pose 9',
										manual_add_pose_to_trajectory(positionX=1.079928, positionY=0.3833965, positionZ=0.19600, orientationX=0, orientationY=0, orientationZ=39.23, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'pose 10'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj8', 'trajectory': 'traj9'})

			# x:907 y:63
			OperatableStateMachine.add('pose4',
										manual_add_pose_to_trajectory(positionX=self.distance_start_loop, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=1, precision=0, long_rotation=False),
										transitions={'continue': 'pose 5'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj2', 'trajectory': 'traj4'})

			# x:511 y:305
			OperatableStateMachine.add('send to planner',
										send_to_planner(),
										transitions={'continue': 'target_reached', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'traj12'})

			# x:190 y:73
			OperatableStateMachine.add('set mode',
										set_control_mode(mode=10, timeout=2),
										transitions={'continue': 'init_traj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:632 y:528
			OperatableStateMachine.add('set mode 0',
										set_control_mode(mode=0, timeout=2),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:893 y:485
			OperatableStateMachine.add('target_reached',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'set mode 0', 'target_not_reached': 'failed', 'error': 'error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:384 y:36
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'pose 1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'empty_traj'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
