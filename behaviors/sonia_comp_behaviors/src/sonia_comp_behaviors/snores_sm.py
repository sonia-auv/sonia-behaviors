#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_navigation_states.send_to_planner import send_to_planner
from sonia_navigation_states.wait_target_reached import wait_target_reached
from sonia_wtf_states.calculate_move_x import CalculateMoveX
from sonia_wtf_states.calculate_move_yz import CalculateMoveYZ
from sonia_wtf_states.get_point_area_AI import GetPointAreaAI
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Aug 06 2023
@author: Nimai
'''
class SnoresSM(Behavior):
	'''
	ZZZzzzZzZ
	'''


	def __init__(self):
		super(SnoresSM, self).__init__()
		self.name = 'Snores'

		# parameters of this behavior
		self.add_parameter('target', 'Glyph_Earth_1')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:675 y:120, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Get Ai',
										GetPointAreaAI(topic="/proc_detection/bounding_box", class_=self.target, needed_imgs=10, timeout=10, screen_width=600, screen_height=400),
										transitions={'success': 'Move YZ', 'timeout': 'failed'},
										autonomy={'success': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'delta_x': 'delta_x', 'delta_y': 'delta_y', 'area': 'area'})

			# x:471 y:39
			OperatableStateMachine.add('Move X',
										CalculateMoveX(target_area=1100, step=0.3, num_steps=3, tolerance=0.1),
										transitions={'success': 'finished', 'move': 'Send'},
										autonomy={'success': Autonomy.Off, 'move': Autonomy.Off},
										remapping={'area': 'area', 'trajectory': 'trajectory'})

			# x:225 y:39
			OperatableStateMachine.add('Move YZ',
										CalculateMoveYZ(target_area=1100, bounding_box_topic="box_target", step=0.3, num_steps=3),
										transitions={'success': 'Move X', 'move': 'Send'},
										autonomy={'success': Autonomy.Off, 'move': Autonomy.Off},
										remapping={'delta_x': 'delta_x', 'delta_y': 'delta_y', 'area': 'area', 'trajectory': 'trajectory'})

			# x:439 y:215
			OperatableStateMachine.add('Send',
										send_to_planner(),
										transitions={'continue': 'Wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:189 y:141
			OperatableStateMachine.add('Wait',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'Get Ai', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
