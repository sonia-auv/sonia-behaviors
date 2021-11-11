#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.get_vision_target import get_vision_target
from sonia_flexbe_states.move_to_target import move_to_target
from sonia_flexbe_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 11 2021
@author: FA
'''
class test_get_filter_targetSM(Behavior):
	'''
	Test the state for the target from filter
	'''


	def __init__(self):
		super(test_get_filter_targetSM, self).__init__()
		self.name = 'test_get_filter_target'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:660 y:48, x:296 y:378
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:93 y:102
			OperatableStateMachine.add('start baby',
										start_filter_chain(param_node_name='simple_body_baby', camera_no=3, param_cmd=1),
										transitions={'continue': 'test', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain'})

			# x:356 y:113
			OperatableStateMachine.add('test',
										get_vision_target(bounding_box_pixel=75, target_width_meter=0.02, target_height_meter=0.08, ratio_victory=0.8, number_of_average=10, camera=3, max_mouvement=2, min_mouvement=0.25),
										transitions={'success': 'finished', 'move': 'move target', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'move': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'pose': 'pose'})

			# x:658 y:230
			OperatableStateMachine.add('move target',
										move_to_target(),
										transitions={'continue': 'test', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
