#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_com_states.synchro_slave import synchro_slave
from sonia_flexbe_behaviors.move_sm import moveSM
from sonia_navigation_states.wait_target_reached import wait_target_reached
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 20 2022
@author: FA
'''
class testsynchroslavepoolSM(Behavior):
	'''
	Wait for the master to send a request. This doesn't have to be a behavior but for testing it is.
	'''


	def __init__(self):
		super(testsynchroslavepoolSM, self).__init__()
		self.name = 'test synchro slave pool'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(moveSM, 'move')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:788 y:406, x:197 y:369
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:144 y:70
			OperatableStateMachine.add('sync',
										synchro_slave(timeout=120),
										transitions={'continue': 'move', 'timeout': 'failed'},
										autonomy={'continue': Autonomy.Off, 'timeout': Autonomy.Off})

			# x:733 y:130
			OperatableStateMachine.add('target_reached',
										wait_target_reached(),
										transitions={'target_reached': 'finished', 'target_not_reached': 'failed', 'error': 'failed'},
										autonomy={'target_reached': Autonomy.Off, 'target_not_reached': Autonomy.Off, 'error': Autonomy.Off})

			# x:426 y:62
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': 0, 'positionY': 0, 'positionZ': 0, 'orientationX': 0, 'orientationY': 0, 'orientationZ': 180, 'frame': 1, 'speed': 0, 'precision': 0, 'rotation': True}),
										transitions={'finished': 'target_reached', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
