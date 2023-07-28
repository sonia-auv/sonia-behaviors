#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_complex_behaviors.align_gate_right_sm import AlignGateRightSM
from sonia_vision_states.start_filter_chain import start_filter_chain
from sonia_vision_states.stop_filter_chain import stop_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 27 2023
@author: Nimai
'''
class StartfilterchainandaligngaterightSM(Behavior):
	'''
	Start the filterchain then align the gate to the right
	'''


	def __init__(self):
		super(StartfilterchainandaligngaterightSM, self).__init__()
		self.name = 'Start filterchain and align gate right'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(AlignGateRightSM, 'Align Gate Right')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:500 y:289, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Start gate detect filterchain',
										start_filter_chain(filterchain="test_gate", target="", camera_no=3),
										transitions={'continue': 'Align Gate Right', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'topic': 'topic', 'filterchain': 'filterchain', 'camera_no': 'camera_no', 'target': 'target'})

			# x:512 y:194
			OperatableStateMachine.add('stop filterchain',
										stop_filter_chain(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no'})

			# x:350 y:66
			OperatableStateMachine.add('Align Gate Right',
										self.use_behavior(AlignGateRightSM, 'Align Gate Right'),
										transitions={'finished': 'stop filterchain', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
