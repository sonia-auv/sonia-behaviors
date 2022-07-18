#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_com_states.choose_your_character import choose_your_character
from sonia_com_states.synchro_receive import synchro_receive
from sonia_com_states.synchro_send import synchro_send
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 18 2022
@author: FA
'''
class test_synchroSM(Behavior):
	'''
	testing synchro with the acknowledge
	'''


	def __init__(self):
		super(test_synchroSM, self).__init__()
		self.name = 'test_synchro'

		# parameters of this behavior
		self.add_parameter('submarine', 'AUV8')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:416 y:244, x:400 y:379
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:146 y:87
			OperatableStateMachine.add('choose_sub',
										choose_your_character(submarine=self.submarine),
										transitions={'auv8': 'send_synchro', 'auv7': 'wait_for_friend'},
										autonomy={'auv8': Autonomy.Off, 'auv7': Autonomy.Off})

			# x:384 y:78
			OperatableStateMachine.add('send_synchro',
										synchro_send(),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:111 y:250
			OperatableStateMachine.add('wait_for_friend',
										synchro_receive(timeout=60),
										transitions={'continue': 'finished', 'timeout': 'failed'},
										autonomy={'continue': Autonomy.Off, 'timeout': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
