#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_hardware_states.wait_mission import wait_mission
from sonia_navigation_states.set_control_mode import set_control_mode
from sonia_navigation_states.trick_shot import trick_shot
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 23 2022
@author: GS
'''
class InitialisationpourtestSM(Behavior):
	'''
	Initialise le sub
	'''


	def __init__(self):
		super(InitialisationpourtestSM, self).__init__()
		self.name = 'Initialisation pour test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:755 y:75, x:262 y:205
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('wait mission switch',
										wait_mission(),
										transitions={'continue': 'set mode', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:521 y:45
			OperatableStateMachine.add('trickshot',
										trick_shot(delay=3),
										transitions={'continue': 'finished'},
										autonomy={'continue': Autonomy.Off})

			# x:299 y:52
			OperatableStateMachine.add('set mode',
										set_control_mode(mode=10, timeout=5),
										transitions={'continue': 'trickshot', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
