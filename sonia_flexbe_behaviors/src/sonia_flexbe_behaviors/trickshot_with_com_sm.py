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
from sonia_com_states.verify_task import verify_task
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
		# x:751 y:160
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:70 y:163
			OperatableStateMachine.add('verify_trickshot',
										verify_task(mission=2, timeout=3),
										transitions={'to_do': 'trickshot', 'skip': 'finished'},
										autonomy={'to_do': Autonomy.Off, 'skip': Autonomy.Off})

			# x:229 y:47
			OperatableStateMachine.add('trickshot',
										trick_shot(delay=15),
										transitions={'continue': 'success_trickshot'},
										autonomy={'continue': Autonomy.Off})

			# x:499 y:48
			OperatableStateMachine.add('success_trickshot',
										send_update(mission=3, state=2),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
