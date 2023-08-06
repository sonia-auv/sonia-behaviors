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
from sonia_vision_states.deltas_to_multipose import DeltasToMultipose
from sonia_vision_states.important_function import ImportantFunction
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Aug 06 2023
@author: Nimai, Victor
'''
class SnorestestSM(Behavior):
	'''
	ZZzzZ Test ZZzzZZ
	'''


	def __init__(self):
		super(SnorestestSM, self).__init__()
		self.name = 'Snores test'

		# parameters of this behavior
		self.add_parameter('target', 'Glyph_Earth_1')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:431 y:529, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:141 y:64
			OperatableStateMachine.add('Shit..',
										ImportantFunction(topic="/proc_detection/bounding_box", class_=self.target, needed_imgs=10, timeout=10, screen_width=600, screen_height=400),
										transitions={'success': 'MOVE', 'timeout': 'failed'},
										autonomy={'success': Autonomy.Off, 'timeout': Autonomy.Off},
										remapping={'delta_x': 'delta_x', 'delta_y': 'delta_y', 'delta_z': 'delta_z'})

			# x:487 y:174
			OperatableStateMachine.add('SEND',
										send_to_planner(),
										transitions={'continue': 'WAIT', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'trajectory'})

			# x:499 y:320
			OperatableStateMachine.add('WAIT',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:335 y:85
			OperatableStateMachine.add('MOVE',
										DeltasToMultipose(),
										transitions={'success': 'SEND'},
										autonomy={'success': Autonomy.Off},
										remapping={'delta_x': 'delta_x', 'delta_y': 'delta_y', 'delta_z': 'delta_z', 'trajectory': 'trajectory'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
