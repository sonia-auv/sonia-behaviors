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
from sonia_hardware_states.motor_test import motor_test
from sonia_hardware_states.wait_mission import wait_mission
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Nov 19 2022
@author: Ewan
'''
class DryTestSM(Behavior):
	'''
	Une mission pour remplacer le dry test du rqt
	'''


	def __init__(self):
		super(DryTestSM, self).__init__()
		self.name = 'Dry Test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:806 y:198, x:491 y:368
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:132 y:34
			OperatableStateMachine.add('mission_switch',
										wait_mission(),
										transitions={'continue': 'motor_test'},
										autonomy={'continue': Autonomy.Off})

			# x:318 y:35
			OperatableStateMachine.add('motor_test',
										motor_test(),
										transitions={'continue': 'droper', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:664 y:31
			OperatableStateMachine.add('torpille',
										activate_io(element=0, side=0, action=1, timeout=8),
										transitions={'continue': 'finished', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:489 y:34
			OperatableStateMachine.add('droper',
										activate_io(element=1, side=0, action=1, timeout=8),
										transitions={'continue': 'torpille', 'failed': 'failed', 'timeout': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
