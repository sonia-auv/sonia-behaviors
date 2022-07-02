#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.add_pose_to_trajectory import add_pose_to_trajectory
from sonia_flexbe_states.init_trajectory import init_trajectory
from sonia_flexbe_states.send_to_planner import send_to_planner
from sonia_flexbe_states.wait_mission import wait_mission
from sonia_flexbe_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed May 11 2022
@author: KY
'''
class Front_gateSM(Behavior):
	'''
	Trajectory - Front of the gate
	'''


	def __init__(self):
		super(Front_gateSM, self).__init__()
		self.name = 'Front_gate'

		# parameters of this behavior
		self.add_parameter('interpolation_method', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:874 y:375, x:130 y:365, x:579 y:366
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'error'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:66 y:38
			OperatableStateMachine.add('mission_switch',
										wait_mission(),
										transitions={'continue': 'init_traj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:479 y:30
			OperatableStateMachine.add('pose_1',
										add_pose_to_trajectory(positionX=20, positionY=3.5, positionZ=1, orientationX=0, orientationY=0, orientationZ=180, frame=1, speed=1, precision=0, rotation=True),
										transitions={'continue': 'send_to_planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'empty_traj', 'trajectory': 'traj_1'})

			# x:672 y:30
			OperatableStateMachine.add('send_to_planner',
										send_to_planner(),
										transitions={'continue': 'target_reached', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'traj_1'})

			# x:909 y:32
			OperatableStateMachine.add('target_reached',
										wait_target_reached(),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'error'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:274 y:34
			OperatableStateMachine.add('init_traj',
										init_trajectory(InterpolationMethod=self.interpolation_method),
										transitions={'continue': 'pose_1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'empty_traj'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
