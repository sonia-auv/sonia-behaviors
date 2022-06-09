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
from sonia_navigation_states.search_zigzag import search_zigzag
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.set_control_mode import set_control_mode
from sonia_navigation_states.wait_target_reached import wait_target_reached
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
		self.add_parameter('boxX', 4)
		self.add_parameter('boxY', 1)
		self.add_parameter('stroke', 1)
		self.add_parameter('radius', 0.2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1225 y:49, x:669 y:291
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:63 y:27
			OperatableStateMachine.add('mode',
										set_control_mode(mode=10, timeout=2),
										transitions={'continue': 'init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:723 y:32
			OperatableStateMachine.add('send',
										send_to_planner(),
										transitions={'continue': 'tr', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:946 y:34
			OperatableStateMachine.add('tr',
										wait_target_reached(),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:509 y:31
			OperatableStateMachine.add('zig',
										search_zigzag(boxX=self.boxX, boxY=self.boxY, stroke=self.stroke, radius=self.radius, side=False),
										transitions={'continue': 'send'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})

			# x:286 y:31
			OperatableStateMachine.add('init',
										init_trajectory(interpolation_method=0),
										transitions={'continue': 'zig'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
