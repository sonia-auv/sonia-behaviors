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
from sonia_com_states.init_mission_list import init_mission_list
from sonia_hardware_states.wait_mission import wait_mission
from sonia_navigation_states.set_control_mode import set_control_mode
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun July 17 2022
@author: FA
'''
class init_submarine_with_comSM(Behavior):
	'''
	Behavior to start the submarine. Set mode of the control, wait for the mission switch and initialise the underwater communication.
	'''


	def __init__(self):
		super(init_submarine_with_comSM, self).__init__()
		self.name = 'init_submarine_with_com'

		# parameters of this behavior
		self.add_parameter('sub_init_array', '1,1,1,0,0,0,0,1,1,1,1')
		self.add_parameter('other_sub_init_array', '1,1,0,1,1,1,1,0,0,0,0')
		self.add_parameter('submarine', 'AUV8')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1047 y:83, x:1052 y:262
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed_start_control'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:115 y:347
			OperatableStateMachine.add('choose_com_mission',
										choose_your_character(submarine=self.submarine),
										transitions={'auv8': 'init_mission_com_auv8', 'auv7': 'init_mission_com_auv7'},
										autonomy={'auv8': Autonomy.Off, 'auv7': Autonomy.Off})

			# x:388 y:479
			OperatableStateMachine.add('init_mission_com_auv7',
										init_mission_list(auv_list=self.other_sub_init_array, other_auv_list=self.sub_init_array, auv='AUV7'),
										transitions={'continue': 'wait for mission switch'},
										autonomy={'continue': Autonomy.Off})

			# x:393 y:298
			OperatableStateMachine.add('init_mission_com_auv8',
										init_mission_list(auv_list=self.sub_init_array, other_auv_list=self.other_sub_init_array, auv='AUV8'),
										transitions={'continue': 'wait for mission switch'},
										autonomy={'continue': Autonomy.Off})

			# x:685 y:161
			OperatableStateMachine.add('set control mode',
										set_control_mode(mode=10, timeout=5),
										transitions={'continue': 'finished', 'failed': 'failed_start_control'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:686 y:385
			OperatableStateMachine.add('wait for mission switch',
										wait_mission(),
										transitions={'continue': 'set control mode'},
										autonomy={'continue': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
