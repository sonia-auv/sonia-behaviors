#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sonia_vision_states.start_filter_chain import start_filter_chain
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed May 11 2022
@author: KY
'''
class onsenfouSM(Behavior):
	'''
	bahhh
	'''


	def __init__(self):
		super(onsenfouSM, self).__init__()
		self.name = 'onsenfou'

		# parameters of this behavior
		self.add_parameter('filterchain_name', 'simple_pipe_straight')
		self.add_parameter('header_name', 'pipe straight')
		self.add_parameter('camera_no', 4)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:916 y:266, x:698 y:87
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:96 y:90
			OperatableStateMachine.add('filter',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=1),
										transitions={'continue': 'stop_filter', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})

			# x:732 y:144
			OperatableStateMachine.add('stop_filter',
										start_filter_chain(param_node_name=self.filterchain_name, header_name=self.header_name, camera_no=self.camera_no, param_cmd=2),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'filterchain': 'filterchain', 'camera_no': 'camera_no', 'header_name': 'header_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
