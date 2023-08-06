#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_hardware_states.wait_mission import wait_mission
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Aug 05 2023
@author: Ewan F
'''
class test_controlSM(Behavior):
	'''
	test des changements de control
	'''


	def __init__(self):
		super(test_controlSM, self).__init__()
		self.name = 'test_control'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(moveSM, 'move')
		self.add_behavior(moveSM, 'move_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:598 y:392, x:276 y:462
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('mission',
										wait_mission(),
										transitions={'continue': 'move'},
										autonomy={'continue': Autonomy.Off})

			# x:177 y:65
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': 10}),
										transitions={'finished': 'wait', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:497 y:172
			OperatableStateMachine.add('move_2',
										self.use_behavior(moveSM, 'move_2',
											parameters={'positionX': -10}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:374 y:105
			OperatableStateMachine.add('wait',
										WaitState(wait_time=5),
										transitions={'done': 'move_2'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
