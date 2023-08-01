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
from sonia_flexbe_states.count_loop import count_loop
from sonia_flexbe_states.init_count_loop import init_count_loop
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 31 2023
@author: Ewan F
'''
class test_loopSM(Behavior):
	'''
	test la limite de loop
	'''


	def __init__(self):
		super(test_loopSM, self).__init__()
		self.name = 'test_loop'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(moveSM, 'move')

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
			# x:102 y:46
			OperatableStateMachine.add('init',
										init_count_loop(),
										transitions={'success': 'move'},
										autonomy={'success': Autonomy.Off},
										remapping={'count_loop': 'count_loop'})

			# x:401 y:121
			OperatableStateMachine.add('loop',
										count_loop(tries=10),
										transitions={'success': 'move', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'count_loop': 'count_loop'})

			# x:106 y:163
			OperatableStateMachine.add('move',
										self.use_behavior(moveSM, 'move'),
										transitions={'finished': 'loop', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
