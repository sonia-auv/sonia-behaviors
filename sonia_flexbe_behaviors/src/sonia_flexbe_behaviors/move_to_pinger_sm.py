#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_hardware_states.get_angle_from_hydro import get_angle_from_hydro
from sonia_hardware_states.locate_pingers import locate_pingers
from sonia_hardware_states.trajectory_to_pinger import trajectory_to_pinger
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 22 2022
@author: GS
'''
class move_to_pingerSM(Behavior):
	'''
	Detect location of pinger and move to it
	'''


	def __init__(self):
		super(move_to_pingerSM, self).__init__()
		self.name = 'move_to_pinger'

		# parameters of this behavior
		self.add_parameter('frequency', '25000')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:356 y:587, x:449 y:317
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:19 y:141
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'get_angle1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:744 y:272
			OperatableStateMachine.add('get_angle2',
										get_angle_from_hydro(frequency=self.frequency, timeout=10, frequency_timeout=10),
										transitions={'continue': 'locate_pinger', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'angle': 'angle2'})

			# x:582 y:155
			OperatableStateMachine.add('is_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'get_angle2', 'moving': 'wait_target', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:59 y:563
			OperatableStateMachine.add('is_moving2',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'wait_target2', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:746 y:400
			OperatableStateMachine.add('locate_pinger',
										locate_pingers(move_dist=1),
										transitions={'continue': 'trajectory_to_pinger', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'angle1': 'angle1', 'angle2': 'angle2', 'real_angle': 'real_angle', 'distance': 'distance'})

			# x:287 y:25
			OperatableStateMachine.add('move_for_locate',
										manual_add_pose_to_trajectory(positionX=1, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:482 y:23
			OperatableStateMachine.add('planner',
										send_to_planner(),
										transitions={'continue': 'wait_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:273 y:446
			OperatableStateMachine.add('planner2',
										send_to_planner(),
										transitions={'continue': 'wait_target2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:455 y:460
			OperatableStateMachine.add('trajectory_to_pinger',
										trajectory_to_pinger(speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'planner2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'real_angle': 'real_angle', 'distance': 'distance', 'trajectory': 'trajectory'})

			# x:677 y:20
			OperatableStateMachine.add('wait_target',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'get_angle2', 'target_not_reached': 'is_moving', 'error': 'is_moving'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:63 y:462
			OperatableStateMachine.add('wait_target2',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:182 y:140
			OperatableStateMachine.add('get_angle1',
										get_angle_from_hydro(frequency=self.frequency, timeout=10, frequency_timeout=10),
										transitions={'continue': 'move_for_locate', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'angle': 'angle1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
