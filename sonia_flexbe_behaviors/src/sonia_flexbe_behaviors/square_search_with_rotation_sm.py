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
from sonia_navigation_states.square_movement import square_movement
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_navigation_states.yaw_orbit_from_given_point import yaw_orbit_from_given_point
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 27 2022
@author: Willy Kao
'''
class square_search_with_rotationSM(Behavior):
	'''
	The sub does an incremental rotation to 360 degrees before every square search.
	'''


	def __init__(self):
		super(square_search_with_rotationSM, self).__init__()
		self.name = 'square_search_with_rotation'

		# parameters of this behavior
		self.add_parameter('boxX', 1.0)
		self.add_parameter('boxY', 1.0)
		self.add_parameter('speed', 2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:482 y:602, x:1156 y:445
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:429 y:31
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'move_square'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_traj'})

			# x:688 y:77
			OperatableStateMachine.add('move_square',
										square_movement(boxX=self.boxX, boxY=self.boxY, speed=self.speed),
										transitions={'continue': 'turn_90_first'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'input_traj', 'trajectory': 'traj_1'})

			# x:688 y:156
			OperatableStateMachine.add('move_square_2',
										square_movement(boxX=self.boxX, boxY=self.boxY, speed=self.speed),
										transitions={'continue': 'turn_90_second'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_2', 'trajectory': 'traj_3'})

			# x:688 y:264
			OperatableStateMachine.add('move_square_3',
										square_movement(boxX=self.boxX, boxY=self.boxY, speed=self.speed),
										transitions={'continue': 'turn_90_third'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_4', 'trajectory': 'traj_5'})

			# x:684 y:346
			OperatableStateMachine.add('move_square_4',
										square_movement(boxX=self.boxX, boxY=self.boxY, speed=self.speed),
										transitions={'continue': 'return_to_initial_yaw'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_6', 'trajectory': 'traj_7'})

			# x:878 y:322
			OperatableStateMachine.add('planner',
										send_to_planner(),
										transitions={'continue': 'wait_target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'traj_7'})

			# x:417 y:414
			OperatableStateMachine.add('return_to_initial_yaw',
										yaw_orbit_from_given_point(pointX=0.16818, pointY=0, rotation=90, speed=1),
										transitions={'continue': 'planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_7', 'trajectory': 'output_traj'})

			# x:417 y:163
			OperatableStateMachine.add('turn_90_first',
										yaw_orbit_from_given_point(pointX=0.16818, pointY=0, rotation=90, speed=1),
										transitions={'continue': 'move_square_2'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_1', 'trajectory': 'traj_2'})

			# x:418 y:253
			OperatableStateMachine.add('turn_90_second',
										yaw_orbit_from_given_point(pointX=0.16818, pointY=0, rotation=90, speed=1),
										transitions={'continue': 'move_square_3'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_3', 'trajectory': 'traj_4'})

			# x:418 y:333
			OperatableStateMachine.add('turn_90_third',
										yaw_orbit_from_given_point(pointX=0.16818, pointY=0, rotation=90, speed=1),
										transitions={'continue': 'move_square_4'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'traj_5', 'trajectory': 'traj_6'})

			# x:879 y:433
			OperatableStateMachine.add('wait_target',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'check_moving', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:880 y:577
			OperatableStateMachine.add('check_moving',
										is_moving(timeout=30, tolerance=0.1),
										transitions={'stopped': 'finished', 'moving': 'wait_target', 'error': 'failed'},
										autonomy={'stopped': Autonomy.Off, 'moving': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
