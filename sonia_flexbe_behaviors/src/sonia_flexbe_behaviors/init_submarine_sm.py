#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.set_control_mode import set_control_mode
from sonia_flexbe_states.set_initial_position import set_initial_position
from sonia_flexbe_states.wait_mission import wait_mission
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 11 2021
@author: FA
'''
class init_submarineSM(Behavior):
	'''
	Behavior to start the submarine. Set mode of the control and wait for the mission switch
	'''


	def __init__(self):
		super(init_submarineSM, self).__init__()
		self.name = 'init_submarine'

		# parameters of this behavior
		self.add_parameter('simulation', False)
		self.add_parameter('mode', 32)
		self.add_parameter('initial_condition_timeout', 2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:948 y:109, x:674 y:301
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:97 y:72
			OperatableStateMachine.add('initial condition',
										set_initial_position(simulation=self.simulation, timeout=self.initial_condition_timeout),
										transitions={'continue': 'set mode '},
										autonomy={'continue': Autonomy.Off})

			# x:363 y:83
			OperatableStateMachine.add('set mode ',
										set_control_mode(mode=self.mode, timeout=5),
										transitions={'continue': 'wait for mission switch', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:616 y:102
			OperatableStateMachine.add('wait for mission switch',
										wait_mission(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]