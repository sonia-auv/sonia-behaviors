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
Created on Jun 23 2022
@author: GS
'''
class move_to_gate_no_trickshot_v2SM(Behavior):
	'''
	Mouvement to gate with trickshot
	'''


	def __init__(self):
		super(move_to_gate_no_trickshot_v2SM, self).__init__()
		self.name = 'move_to_gate_no_trickshot_v2'

		# parameters of this behavior
		self.add_parameter('distance_to_gate', 5)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1315 y:51, x:720 y:158
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:50 y:27
			OperatableStateMachine.add('init traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'move to gate'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:283 y:31
			OperatableStateMachine.add('move to gate',
										manual_add_pose_to_trajectory(positionX=self.distance_to_gate, positionY=0, positionZ=0, orientationX=0, orientationY=0, orientationZ=0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'send to planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:542 y:33
			OperatableStateMachine.add('send to planner',
										send_to_planner(),
										transitions={'continue': 'wait target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:1083 y:383
			OperatableStateMachine.add('stop',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'wait target', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:782 y:32
			OperatableStateMachine.add('wait target',
										wait_target_reached(timeout=30),
										transitions={'target_reached': 'finished', 'target_not_reached': 'stop', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
