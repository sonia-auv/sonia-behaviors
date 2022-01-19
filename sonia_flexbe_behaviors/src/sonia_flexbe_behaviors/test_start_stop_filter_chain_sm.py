#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.start_filter_chain import start_filter_chain
from sonia_flexbe_states.wait_mission import wait_mission
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Nov 13 2021
@author: William Brouillard
'''
class test_start_stop_filter_chainSM(Behavior):
	'''
	Start the filter chain, move for 1 min then stop the filter chain.
	'''


	def __init__(self):
		super(test_start_stop_filter_chainSM, self).__init__()
		self.name = 'test_start_stop_filter_chain'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:573 y:310, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:123 y:66
			OperatableStateMachine.add('Start_filter_chain',
										start_filter_chain(param_node_name='simple_pipe_45', header_name=oof, camera_no=1, param_cmd=1),
										transitions={'continue': 'wait_for_mission_switch', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'bebe', 'camera_no': 'front', 'header_name': 'header_name'})

			# x:511 y:160
			OperatableStateMachine.add('Stop_filter_chain',
										start_filter_chain(param_node_name='simple_pipe_45', header_name=off, camera_no=1, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'bebe', 'camera_no': 'front', 'header_name': 'header_name'})

			# x:317 y:74
			OperatableStateMachine.add('wait_for_mission_switch',
										wait_mission(),
										transitions={'continue': 'Stop_filter_chain', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
