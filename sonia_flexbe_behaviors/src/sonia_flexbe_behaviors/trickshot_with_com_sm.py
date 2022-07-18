#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_com_states.send_update import send_update
from sonia_navigation_states.trick_shot import trick_shot
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Jul 17 2022
@author: FA
'''
class TrickshotwithcomSM(Behavior):
	'''
	Trickshot state with the underwater communication.
	'''


	def __init__(self):
		super(TrickshotwithcomSM, self).__init__()
		self.name = 'Trickshot with com'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:757 y:70
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:170 y:57
			OperatableStateMachine.add('trickshot',
										trick_shot(delay=15),
										transitions={'continue': 'success_trickshot'},
										autonomy={'continue': Autonomy.Off})

			# x:476 y:56
			OperatableStateMachine.add('success_trickshot',
										send_update(mission=3, state=2),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
