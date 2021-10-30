#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Oct 30 2021
@author: Alex
'''
class TestAlexSM(Behavior):
	'''
	TestAlex
	'''


	def __init__(self):
		super(TestAlexSM, self).__init__()
		self.name = 'TestAlex'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:649 y:82, x:639 y:171
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:73 y:76
			OperatableStateMachine.add('Test',
										LogState(text='TEST', severity=Logger.REPORT_HINT),
										transitions={'done': 'Test2'},
										autonomy={'done': Autonomy.Off})

			# x:253 y:77
			OperatableStateMachine.add('Test2',
										LogState(text="Alex", severity=Logger.REPORT_HINT),
										transitions={'done': 'Log'},
										autonomy={'done': Autonomy.Off})

			# x:455 y:147
			OperatableStateMachine.add('Log',
										LogState(text="LOG", severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
