#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_com_states.send_update import send_update
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 17 2022
@author: FA
'''
class GatewithcomSM(Behavior):
	'''
	Move forward through the gate with the underwater communication
	'''


	def __init__(self):
		super(GatewithcomSM, self).__init__()
		self.name = 'Gate with com'

		# parameters of this behavior
		self.add_parameter('distance_to_gate', 3.5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:957 y:547, x:54 y:436
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:443 y:59
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'move_to_gate'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_traj'})

			# x:178 y:518
			OperatableStateMachine.add('failed_gate',
										send_update(mission=1, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})

			# x:444 y:381
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'failed_gate'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:428 y:210
			OperatableStateMachine.add('move_to_gate',
										manual_add_pose_to_trajectory(positionX=self.distance_to_gate, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'input_traj', 'trajectory': 'trajectory'})

			# x:738 y:541
			OperatableStateMachine.add('success_gate',
										send_update(mission=1, state=2),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:444 y:495
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'success_gate', 'target_not_reached': 'check_moving', 'error': 'failed_gate'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:454 y:622
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'success_gate', 'moving': 'wait', 'error': 'failed_gate'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
