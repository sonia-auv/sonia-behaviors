#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.move_sm import moveSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Feb 02 2022
@author: FA
'''
class test_moveSM(Behavior):
	'''
	Testing the behaviors for a single move
	'''


	def __init__(self):
		super(test_moveSM, self).__init__()
		self.name = 'test_move'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(moveSM, 'move')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:294, x:229 y:298
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:100 y:82
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move',
											parameters={'positionX': 9, 'positionY': 16, 'positionZ': 34, 'orientationX': 3, 'orientationY': 7, 'orientationZ': 12, 'frame': 3, 'speed': 1, 'precision': 4, 'rotation': True}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
