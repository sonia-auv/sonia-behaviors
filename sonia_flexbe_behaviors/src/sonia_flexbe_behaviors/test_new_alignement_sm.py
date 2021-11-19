#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.stop_move import stop_move
from sonia_flexbe_states.verify_centroid import verify_centroid
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 18 2021
@author: FA
'''
class testnewalignementSM(Behavior):
	'''
	Test a simple algorithm for the alignement
	'''


	def __init__(self):
		super(testnewalignementSM, self).__init__()
		self.name = 'test new alignement'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:755 y:90, x:367 y:300, x:559 y:289
		_state_machine = OperatableStateMachine(outcomes=['align_successed', 'timeout_reached', 'failed'], input_keys=['filterchain', 'bounding_box', 'header_name'])
		_state_machine.userdata.filterchain = ' '
		_state_machine.userdata.bounding_box = 150
		_state_machine.userdata.header_name = '  '

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:149 y:67
			OperatableStateMachine.add('Check centroid',
										verify_centroid(number_sample=10, timeout=30),
										transitions={'align_complete': 'stop move', 'timeout_reached': 'timeout_reached'},
										autonomy={'align_complete': Autonomy.Off, 'timeout_reached': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'bounding_box': 'bounding_box', 'header_name': 'header_name'})

			# x:453 y:62
			OperatableStateMachine.add('stop move',
										stop_move(timeout=10),
										transitions={'continue': 'align_successed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
