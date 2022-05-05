#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.init_trajectory import init_trajectory
from sonia_flexbe_states.send_to_planner import send_to_planner
from sonia_flexbe_states.set_control_mode import set_control_mode
from sonia_flexbe_states.wait_target_reached import wait_target_reached
from sonia_flexbe_states.zigzag_search import zizag_search
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu May 05 2022
@author: lamarre
'''
class test_zigzagSM(Behavior):
	'''
	test zigzag state
	'''


	def __init__(self):
		super(test_zigzagSM, self).__init__()
		self.name = 'test_zigzag'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1173 y:53, x:1160 y:240
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:90 y:45
			OperatableStateMachine.add('mode',
										set_control_mode(mode=10, timeout=5),
										transitions={'continue': 'init_traj', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:672 y:57
			OperatableStateMachine.add('send',
										send_to_planner(),
										transitions={'continue': 'target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:947 y:59
			OperatableStateMachine.add('target',
										wait_target_reached(),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:464 y:53
			OperatableStateMachine.add('zigzag',
										zizag_search(boxX=5, boxY=5, stroke=1, radius=0.4, side=True),
										transitions={'continue': 'send'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:279 y:54
			OperatableStateMachine.add('init_traj',
										init_trajectory(InterpolationMethod=0),
										transitions={'continue': 'zigzag'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
