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
from sonia_vision_states.calc_ctr_ai import calculate_ctr_ai
from sonia_vision_states.get_ai import get_ai
from sonia_vision_states.move_to_ctr import move_to_ctr
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Aug 05 2023
@author: ewan
'''
class bullshitSM(Behavior):
	'''
	test ai control
	'''


	def __init__(self):
		super(bullshitSM, self).__init__()
		self.name = 'bullshit'

		# parameters of this behavior
		self.add_parameter('target', 'Glyph_Earth_1')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:473 y:511, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('qwerty',
										get_ai(class_name=self.target, nb_img=5),
										transitions={'success': 'ngbfvdcsx', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ai_pos': 'ai_pos'})

			# x:459 y:264
			OperatableStateMachine.add('send',
										send_to_planner(),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'input_traj': 'init_traj'})

			# x:484 y:376
			OperatableStateMachine.add('wait',
										wait_target_reached(timeout=5),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:479 y:186
			OperatableStateMachine.add('xdcfvgbhjn',
										move_to_ctr(),
										transitions={'success': 'send'},
										autonomy={'success': Autonomy.Off},
										remapping={'obj_ctr': 'obj_ctr', 'init_traj': 'init_traj'})

			# x:192 y:105
			OperatableStateMachine.add('ngbfvdcsx',
										calculate_ctr_ai(),
										transitions={'success': 'xdcfvgbhjn'},
										autonomy={'success': Autonomy.Off},
										remapping={'ai_pos': 'ai_pos', 'obj_ctr': 'obj_ctr'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
