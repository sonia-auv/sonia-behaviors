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
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_navigation_states.yaw_orbit_from_given_point_and_angle import yaw_orbit_from_given_point_and_angle
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jul 30 2022
@author: CS
'''
class path2SM(Behavior):
	'''
	Invert direction after getting path 1 orientation
	'''


	def __init__(self):
		super(path2SM, self).__init__()
		self.name = 'path2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:878 y:46, x:523 y:472
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['angle'])
		_state_machine.userdata.angle = 0
		_state_machine.userdata.camera = 2

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:190 y:94
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'rotate_from_path1'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:465 y:117
			OperatableStateMachine.add('move',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:190 y:239
			OperatableStateMachine.add('rotate_from_path1',
										yaw_orbit_from_given_point_and_angle(pointX=0, pointY=0),
										transitions={'continue': 'move'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'camera': 'camera', 'angle': 'angle', 'trajectory': 'trajectory'})

			# x:715 y:175
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'check', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:1000 y:262
			OperatableStateMachine.add('check',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'wait', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
