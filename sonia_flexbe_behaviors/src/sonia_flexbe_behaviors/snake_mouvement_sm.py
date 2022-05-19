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
Created on Mon Nov 15 2021
@author: FA
'''
class snake_mouvementSM(Behavior):
	'''
	Search mouvement in a snake partern. Includes 7 moves with a time entered in parameter
	'''


	def __init__(self):
		super(snake_mouvementSM, self).__init__()
		self.name = 'snake_mouvement'

		# parameters of this behavior
		self.add_parameter('distance_y', 2)
		self.add_parameter('timeout', 25)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1090 y:64, x:1201 y:206
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:19 y:206
			OperatableStateMachine.add('control mode',
										set_control_mode(mode=10, timeout=2),
										transitions={'continue': 'init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:186 y:135
			OperatableStateMachine.add('init',
										init_trajectory(InterpolationMethod=0),
										transitions={'continue': 'zigzag'},
										autonomy={'continue': Autonomy.Off},
										remapping={'trajectory': 'trajectory'})

			# x:471 y:90
			OperatableStateMachine.add('send_planner',
										send_to_planner(),
										transitions={'continue': 'wait target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:665 y:100
			OperatableStateMachine.add('wait target',
										wait_target_reached(),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:294 y:43
			OperatableStateMachine.add('zigzag',
										search_zigzag(boxX=5, boxY=5, stroke=1, radius=0.4, side=False),
										transitions={'continue': 'send_planner'},
										autonomy={'continue': Autonomy.Off},
										remapping={'input_traj': 'trajectory', 'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
