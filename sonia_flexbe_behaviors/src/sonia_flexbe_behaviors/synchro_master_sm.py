#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_com_states.synchro_master import synchro_master
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jun 08 2022
@author: FA
'''
class SynchroMasterSM(Behavior):
	'''
	Test du state pour la synchronisation du master
	'''


	def __init__(self):
		super(SynchroMasterSM, self).__init__()
		self.name = 'Synchro Master'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:574 y:344, x:579 y:186, x:572 y:57
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'timeout'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:157 y:64
			OperatableStateMachine.add('sync',
										synchro_master(mission_id=2, timeout=300),
										transitions={'continue': 'finished', 'failed': 'failed', 'timeout': 'timeout'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'timeout': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
