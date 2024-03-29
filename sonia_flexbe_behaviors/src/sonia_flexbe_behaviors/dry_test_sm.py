#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_hardware_states.activate_io import activate_io
from sonia_hardware_states.wait_mission import wait_mission
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 27 2022
@author: sonia
'''
class dry_testSM(Behavior):
	'''
	dry test the mission/kill switch by dropping droppers
	'''


	def __init__(self):
		super(dry_testSM, self).__init__()
		self.name = 'dry_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:79 y:102
			OperatableStateMachine.add('test',
										wait_mission(),
										transitions={'continue': 'io'},
										autonomy={'continue': Autonomy.Off})

			# x:358 y:91
			OperatableStateMachine.add('io',
										activate_io(element=1, side=0, action=1, timeout=8),
										transitions={'continue': 'finished', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
