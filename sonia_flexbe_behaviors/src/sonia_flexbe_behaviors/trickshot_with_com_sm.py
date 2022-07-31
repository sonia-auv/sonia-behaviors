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
from sonia_flexbe_behaviors.trickshot_sm import trickshotSM
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
		self.add_parameter('has_com', True)

		# references to used behaviors
		self.add_behavior(trickshotSM, 'trickshot')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:746 y:340, x:647 y:270
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:66 y:184
			OperatableStateMachine.add('verify_trickshot',
										verify_task(mission=2, has_com=self.has_com, timeout=3),
										transitions={'to_do': 'trickshot', 'skip': 'finished'},
										autonomy={'to_do': Autonomy.Off, 'skip': Autonomy.Off})

			# x:626 y:71
			OperatableStateMachine.add('success_trickshot',
										send_update(mission=2, state=2),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:235 y:46
			OperatableStateMachine.add('trickshot',
										self.use_behavior(trickshotSM, 'trickshot'),
										transitions={'finished': 'success_trickshot', 'failed': 'Failed_trickshot'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:455 y:163
			OperatableStateMachine.add('Failed_trickshot',
										send_update(mission=2, state=-1),
										transitions={'continue': 'failed'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
