#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.coin_flip_gate__notrickshot_task_sm import coin_flip_gate_notrickshot_taskSM
from sonia_navigation_states.trick_shot import trick_shot
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jun 24 2022
@author: GS
'''
class coin_flip_gate_withtrickshot_taskSM(Behavior):
	'''
	Orient to gate for coin flip task and move forward through the gate with trickshot
	'''


	def __init__(self):
		super(coin_flip_gate_withtrickshot_taskSM, self).__init__()
		self.name = 'coin_flip_gate_ withtrickshot_task'

		# parameters of this behavior
		self.add_parameter('orientation_to_gate', 0)
		self.add_parameter('dive_depth', 1)
		self.add_parameter('distance_to_gate', 4)

		# references to used behaviors
		self.add_behavior(coin_flip_gate_notrickshot_taskSM, 'coin_flip_gate_ notrickshot_task')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:602 y:128, x:163 y:278
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:81 y:114
			OperatableStateMachine.add('coin_flip_gate_ notrickshot_task',
										self.use_behavior(coin_flip_gate_notrickshot_taskSM, 'coin_flip_gate_ notrickshot_task'),
										transitions={'finished': 'trickshot', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:375 y:113
			OperatableStateMachine.add('trickshot',
										trick_shot(delay=15),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
