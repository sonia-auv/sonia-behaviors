#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_comp_behaviors.bullshit_magic_gate_sm import BullshitMagicgateSM
from sonia_comp_behaviors.snores_poke_sm import SnorespokeSM
from sonia_hardware_states.wait_mission import wait_mission
from sonia_navigation_states.set_control_mode import set_control_mode
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Aug 06 2023
@author: Niami Victor Ewa
'''
class Runsemifinals2023SM(Behavior):
	'''
	At this point, fuck it, we ball
	'''


	def __init__(self):
		super(Runsemifinals2023SM, self).__init__()
		self.name = 'Run semi-finals 2023'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(BullshitMagicgateSM, 'Bullshit Magic gate')
		self.add_behavior(SnorespokeSM, 'Snores poke')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:458, x:812 y:165
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:168 y:69
			OperatableStateMachine.add('mission',
										wait_mission(),
										transitions={'continue': 'Bullshit Magic gate'},
										autonomy={'continue': Autonomy.Off})

			# x:374 y:198
			OperatableStateMachine.add('Snores poke',
										self.use_behavior(SnorespokeSM, 'Snores poke'),
										transitions={'finished': 'finished', 'failed': 'kill'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'gate_side': 'gate_side'})

			# x:619 y:93
			OperatableStateMachine.add('kill',
										set_control_mode(mode=0, timeout=5),
										transitions={'continue': 'failed', 'failed': 'kill'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:374 y:100
			OperatableStateMachine.add('Bullshit Magic gate',
										self.use_behavior(BullshitMagicgateSM, 'Bullshit Magic gate'),
										transitions={'finished': 'Snores poke', 'failed': 'kill'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'gate_side': 'gate_side'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
