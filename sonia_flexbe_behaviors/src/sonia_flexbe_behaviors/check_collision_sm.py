#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.touch_buoy_sm import touch_buoySM
from sonia_navigation_states.has_collided import has_collided
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.is_moving import is_moving
from sonia_navigation_states.manual_add_pose_to_trajectory import manual_add_pose_to_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.stop_move import stop_move
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 15 2022
@author: GS
'''
class check_collisionSM(Behavior):
	'''
	Move forward and check collision
	'''


	def __init__(self):
		super(check_collisionSM, self).__init__()
		self.name = 'check_collision'

		# parameters of this behavior
		self.add_parameter('threshold', 0.2)

		# references to used behaviors
		self.add_behavior(touch_buoySM, 'Container/touch_buoy')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:461 y:367, x:1269 y:430
		_state_machine = OperatableStateMachine(outcomes=['failed', 'target_reached'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:458, x:191 y:451, x:779 y:39, x:330 y:458, x:430 y:458
		_sm_container_0 = ConcurrencyContainer(outcomes=['failed', 'target_reached'], conditions=[
										('failed', [('touch_buoy', 'failed'), ('has_collided', 'error')]),
										('target_reached', [('has_collided', 'target_reached')]),
										('failed', [('touch_buoy', 'finished')])
										])

		with _sm_container_0:
			# x:143 y:27
			OperatableStateMachine.add('touch_buoy',
										self.use_behavior(touch_buoySM, 'Container/touch_buoy'),
										transitions={'finished': 'failed', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:494 y:114
			OperatableStateMachine.add('has_collided',
										has_collided(timeout=30, threshold=self.threshold),
										transitions={'target_reached': 'target_reached', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'error': Autonomy.Off})



		with _state_machine:
			# x:93 y:36
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'failed': 'failed', 'target_reached': 'stop'},
										autonomy={'failed': Autonomy.Inherit, 'target_reached': Autonomy.Inherit})

			# x:920 y:525
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'target_reached', 'moving': 'final_point', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})

			# x:918 y:357
			OperatableStateMachine.add('final_point',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'target_reached', 'target_not_reached': 'check_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:657 y:45
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'move_backward'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:900 y:44
			OperatableStateMachine.add('move_backward',
										manual_add_pose_to_trajectory(positionX=-2, positionY=0.0, positionZ=0.0, orientationX=0.0, orientationY=0.0, orientationZ=0.0, frame=1, speed=0, precision=0, long_rotation=False),
										transitions={'continue': 'planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:904 y:201
			OperatableStateMachine.add('planner',
										send_to_planner(),
										transitions={'continue': 'final_point', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:344 y:45
			OperatableStateMachine.add('stop',
										stop_move(timeout=30),
										transitions={'target_reached': 'init', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
