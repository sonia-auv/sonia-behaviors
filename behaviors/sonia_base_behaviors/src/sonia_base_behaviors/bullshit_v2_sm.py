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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Aug 05 2023
@author: ewan
'''
class bullshit_v2SM(Behavior):
	'''
	test ai control
	'''


	def __init__(self):
		super(bullshit_v2SM, self).__init__()
		self.name = 'bullshit_v2'

		# parameters of this behavior
		self.add_parameter('target', 'Glyph_Earth_1')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:242 y:319, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['obj_ctr'])
		_state_machine.userdata.obj_ctr = []

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

			# x:192 y:105
			OperatableStateMachine.add('ngbfvdcsx',
										calculate_ctr_ai(),
										transitions={'success': 'finished'},
										autonomy={'success': Autonomy.Off},
										remapping={'ai_pos': 'ai_pos', 'obj_ctr': 'obj_ctr', 'obj_area': 'obj_area'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
