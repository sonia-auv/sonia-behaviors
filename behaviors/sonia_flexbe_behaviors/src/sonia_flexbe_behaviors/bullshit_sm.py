#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
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

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:529 y:373, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('qwerty',
										get_ai(class_name="Glyph_Earth_2", nb_img=2),
										transitions={'success': 'ngbfvdcsx', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ai_pos': 'ai_pos'})

			# x:479 y:186
			OperatableStateMachine.add('xdcfvgbhjn',
										move_to_ctr(),
										transitions={'success': 'finished'},
										autonomy={'success': Autonomy.Off},
										remapping={'obj_ctr': 'obj_ctr'})

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
