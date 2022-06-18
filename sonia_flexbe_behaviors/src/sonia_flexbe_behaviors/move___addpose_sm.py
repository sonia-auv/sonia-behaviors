#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_navigation_states.add_addpose_to_trajectory import add_addpose_to_trajectory
from sonia_navigation_states.init_trajectory import init_trajectory
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 14 2022
@author: FA
'''
class MoveAddposeSM(Behavior):
	'''
	This behavior use the msg Addpose to create the needed trajectory. This should only be use for the vision. The behavior "Move" should be use to create a single mouvement.
	'''


	def __init__(self):
		super(MoveAddposeSM, self).__init__()
		self.name = 'Move - Addpose'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:833 y:390, x:1133 y:90, x:1133 y:290
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'failed_target_reached'], input_keys=['pose'])
		_state_machine.userdata.pose = ' '

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:133 y:67
			OperatableStateMachine.add('init_traj',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'add_addpose_msg'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'input_traj'})

			# x:733 y:67
			OperatableStateMachine.add('send_to_planner',
										send_to_planner(),
										transitions={'continue': 'target_reached', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:733 y:202
			OperatableStateMachine.add('target_reached',
										wait_target_reached(),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed_target_reached', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:426 y:67
			OperatableStateMachine.add('add_addpose_msg',
										add_addpose_to_trajectory(),
										transitions={'continue': 'send_to_planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'input_traj', 'pose': 'pose', 'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
