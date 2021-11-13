#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_flexbe_states.move_single import move_single
from sonia_flexbe_states.start_filter_chain import start_filter_chain
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
		# x:456 y:307, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:123 y:66
			OperatableStateMachine.add('Start_filter_chain',
										start_filter_chain(param_node_name=jiangshi, camera_no=1, param_cmd=1),
										transitions={'continue': 'move', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'jiangshi', 'camera_no': 'front'})

			# x:497 y:139
			OperatableStateMachine.add('Stop_filter_chain',
										start_filter_chain(param_node_name=jiangshi, camera_no=1, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'jiangshi', 'camera_no': 'front'})

			# x:323 y:67
			OperatableStateMachine.add('move',
										move_single(positionX=20, positionY=0, positionZ=1, orientationX=0, orientationY=0, orientationZ=0, frame=2, time=60, precision=0, rotation=True),
										transitions={'continue': 'Stop_filter_chain', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
