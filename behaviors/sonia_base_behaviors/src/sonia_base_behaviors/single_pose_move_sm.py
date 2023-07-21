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
Created on Fri Jul 21 2023
@author: Nimai
'''
class SinglePoseMoveSM(Behavior):
	'''
	Move a single Pose
	'''


	def __init__(self):
		super(SinglePoseMoveSM, self).__init__()
		self.name = 'Single Pose Move'

		# parameters of this behavior
		self.add_parameter('positionX', 0.0)
		self.add_parameter('positionY', 0.0)
		self.add_parameter('positionZ', 0.0)
		self.add_parameter('orientationZ', 0.0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:471 y:409, x:322 y:352
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:114 y:56
			OperatableStateMachine.add('init_trag',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'Pose to move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_traj'})

			# x:317 y:74
			OperatableStateMachine.add('Send move',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'input_traj'})

			# x:453 y:231
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:109 y:191
			OperatableStateMachine.add('Pose to move',
										manual_add_pose_to_trajectory(positionX=self.positionX, positionY=self.positionY, positionZ=self.positionZ, orientationX=0.0, orientationY=0.0, orientationZ=self.orientationZ, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'Send move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'input_traj', 'trajectory': 'input_traj'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
