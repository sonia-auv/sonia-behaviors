#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_hardware_states.module_check import module_check
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat May 28 2022
@author: FA
'''
class testprocfaultSM(Behavior):
	'''
	Test the fault detection of different modules
	'''


	def __init__(self):
		super(testprocfaultSM, self).__init__()
		self.name = 'test proc fault'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:138 y:393, x:500 y:419
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:198 y:89
			OperatableStateMachine.add('module check',
										module_check(navigation=True, vision=False, mapping=False, hydro=False, io=False, hardware=True),
										transitions={'continue': 'finished', 'reboot': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'reboot': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
