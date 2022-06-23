#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_behaviors.gate_no_trickshot_task_v2_sm import gate_no_trickshot_task_v2SM
from sonia_navigation_states.trick_shot import trick_shot
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 15 2021
@author: FA
'''
class move_to_gate_trickshot_v2SM(Behavior):
	'''
	Mouvement to gate with trickshot
	'''


	def __init__(self):
		super(move_to_gate_trickshot_v2SM, self).__init__()
		self.name = 'move_to_gate_trickshot_v2'

		# parameters of this behavior
		self.add_parameter('distance_to_gate', 5)

		# references to used behaviors
		self.add_behavior(gate_no_trickshot_task_v2SM, 'gate_no_trickshot_task_v2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:846 y:87, x:398 y:199
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:113 y:48
			OperatableStateMachine.add('gate_no_trickshot_task_v2',
										self.use_behavior(gate_no_trickshot_task_v2SM, 'gate_no_trickshot_task_v2'),
										transitions={'finished': 'trickshot', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:575 y:69
			OperatableStateMachine.add('trickshot',
										trick_shot(delay=3),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
