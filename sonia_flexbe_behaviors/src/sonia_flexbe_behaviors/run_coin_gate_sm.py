#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.init_sim_sm import init_simSM
from sonia_flexbe_behaviors.move_coin_filp_sm import move_coin_filpSM
from sonia_flexbe_behaviors.move_to_gate_sm import move_to_gateSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 11 2021
@author: FA
'''
class run_coin_gateSM(Behavior):
	'''
	Test run for coin filp and gate
	'''


	def __init__(self):
		super(run_coin_gateSM, self).__init__()
		self.name = 'run_coin_gate'

		# parameters of this behavior
		self.add_parameter('simulation', True)

		# references to used behaviors
		self.add_behavior(init_simSM, 'init_sim')
		self.add_behavior(move_coin_filpSM, 'move_coin_filp')
		self.add_behavior(move_to_gateSM, 'move_to_gate')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:885 y:341, x:426 y:445
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:109 y:152
			OperatableStateMachine.add('init_sim',
										self.use_behavior(init_simSM, 'init_sim'),
										transitions={'finished': 'move_coin_filp', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:382 y:165
			OperatableStateMachine.add('move_coin_filp',
										self.use_behavior(move_coin_filpSM, 'move_coin_filp'),
										transitions={'finished': 'move_to_gate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:674 y:173
			OperatableStateMachine.add('move_to_gate',
										self.use_behavior(move_to_gateSM, 'move_to_gate'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
