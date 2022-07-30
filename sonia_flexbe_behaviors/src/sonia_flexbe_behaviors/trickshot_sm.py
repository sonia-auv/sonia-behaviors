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
@author: KY-GS
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
		# x:30 y:365, x:786 y:382
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

			# x:1240 y:93
			OperatableStateMachine.add('init_roue',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'traj2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:833 y:635
			OperatableStateMachine.add('int traj 3',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'roll1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:1025 y:163
			OperatableStateMachine.add('is move',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'init_roue', 'moving': 'tg', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:1232 y:641
			OperatableStateMachine.add('is_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'int traj 3', 'moving': 'traget_reached', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:200 y:288
			OperatableStateMachine.add('ismove3',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'TG3', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:612 y:642
			OperatableStateMachine.add('roll1',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=180.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'roll2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:363 y:631
			OperatableStateMachine.add('roll2',
										manual_add_pose_to_trajectory(positionX=0.0, positionY=0.0, positionZ=0.0, orientationX=180.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:557 y:73
			OperatableStateMachine.add('send1',
										send_to_planner(),
										transitions={'continue': 'tg', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'traj'})

			# x:191 y:621
			OperatableStateMachine.add('send3',
										send_to_planner(),
										transitions={'continue': 'TG3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:1227 y:470
			OperatableStateMachine.add('send_to_planner',
										send_to_planner(),
										transitions={'continue': 'traget_reached', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'traj'})

			# x:798 y:73
			OperatableStateMachine.add('tg',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'init_roue', 'target_not_reached': 'is move', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:1228 y:554
			OperatableStateMachine.add('traget_reached',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'int traj 3', 'target_not_reached': 'is_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:270 y:82
			OperatableStateMachine.add('traj1',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=90, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'new_traj', 'trajectory': 'traj'})

			# x:1212 y:230
			OperatableStateMachine.add('traj2',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=180, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'traj3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'traj'})

			# x:1212 y:303
			OperatableStateMachine.add('traj3',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=180, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'traj4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj', 'trajectory': 'traj'})

			# x:1211 y:390
			OperatableStateMachine.add('traj4',
										manual_add_pose_to_trajectory(positionX=0, positionY=0, positionZ=0, orientationX=0, orientationY=-90, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send_to_planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj', 'trajectory': 'traj'})

			# x:196 y:477
			OperatableStateMachine.add('TG3',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'ismove3', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
